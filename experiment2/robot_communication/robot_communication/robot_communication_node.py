from math import cos, sin, pi
import numpy as np
import socket
import subprocess
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String
from naoqi_bridge_msgs.msg import JointAnglesWithSpeed
from dqrobotics import *
from dqrobotics.solvers import DQ_QuadprogSolver
from dqrobotics.robot_control import DQ_ClassicQPController, ControlObjective
from rclpy.qos import (QoSProfile, ReliabilityPolicy, HistoryPolicy,
                       DurabilityPolicy)
from rclpy.callback_groups import (ReentrantCallbackGroup,
                                   MutuallyExclusiveCallbackGroup)
from rclpy.executors import MultiThreadedExecutor
from ament_index_python.packages import get_package_share_directory

from robot_communication import NaoRobot
from package_with_interfaces.msg import (IntArray, ObjectIndication, NaoCommand,
                                         FacialExpression)


class RobotCommunicationNode(Node):

    def __init__(self):
        super().__init__('robot_communication_node')

        # Getting parameters from launch file
        # Robot IP address
        self.declare_parameter("robot_ip", rclpy.Parameter.Type.STRING)
        self.nao_ip: str = self.get_parameter(
            'robot_ip').get_parameter_value().string_value
        # List with information about the objects/points of interest
        # For each object, there is the object ID, a marker ID, and the
        # x, y, and z coordinates of the object relative to the marker
        # objects = [[object_ID, marker_ID, [x, y, z]],  # first object
        #            [object_ID, marker_ID, [x, y, z]]]  # second object
        #            ...]
        self.objects = []
        for i in range(1, 500):
            try:
                name = "object" + str(i)
                self.declare_parameter(name, rclpy.Parameter.Type.DOUBLE_ARRAY)
                object = self.get_parameter(
                    name).get_parameter_value().double_array_value
                object = [int(object[0]), int(object[1]), object[2:5]]
                self.objects.append(object)
            except:
                if i == 2:
                    message = f"{i - 1} object defined"
                    self.get_logger().info("\033[32m{}\033[0m".format(message))
                else:
                    message = f"{i - 1} objects defined"
                    self.get_logger().info("\033[32m{}\033[0m".format(message))

                break

        # Getting parameters to connect with the robot server
        self.declare_parameter("ip_robot_server", rclpy.Parameter.Type.STRING)
        try:
            self.ip_server: str = self.get_parameter(
                'ip_robot_server').get_parameter_value().string_value
        except:
            self.ip_server = '127.0.0.1'
        self.declare_parameter("port_robot_server",
                               rclpy.Parameter.Type.INTEGER)
        try:
            self.port_server: int = (self.get_parameter('port_robot_server').
                                     get_parameter_value().integer_value)
        except:
            self.port_server = 1111

        # Starting an auxiliary server connected to the robot
        path = get_package_share_directory("robot_communication")
        path = (path.split("install")[0] +
                "src/robot_communication/robot_communication/")
        file = "robot_server.py"
        self.robot_server = subprocess.Popen(["python2.7", path + file,
                                              self.nao_ip])
        time.sleep(3)

        # Disabling autonomous behaviors
        response = self.send_server_request("AutonomousLife getState",
                                          response=True)
        if 'disabled' not in response:
            self.send_server_request("AutonomousLife setState disabled")

        # List of tracked markers IDs
        self.tracked_markers = []

        # Maximum number of markers
        # (defined by the Aruco dictionary used in detection)
        self.max_marker = 50

        # List of markers poses relative to marker 0
        self.markers_from_zero = [DQ([1])]*(self.max_marker - 1)

        # Creating subscribers for markers poses relative to marker 0
        for i in range(0, self.max_marker):
            topic_name = "/x0_" + str(i)
            self.create_subscription(
                msg_type=PoseStamped,
                topic=topic_name,
                callback=lambda msg, id=i: self.markers_callback(msg, id),
                qos_profile=1)

        # Creating subscriber for the list of tracked markers
        self.create_subscription(msg_type=IntArray,
                                 topic="/tracked_markers_ids",
                                 callback=self.tracked_ids_callback,
                                 qos_profile=1)

        # Transformation from marker 0 to the robot torso
        rx = cos((-pi/2)/2) + DQ.i*sin((-pi/2)/2)
        rz = cos((-pi/2)/2) + DQ.k*sin((-pi/2)/2)
        t = 1 + DQ.E*0.5*DQ([-0.049871, -0.001588, -0.055163])
        self.x0_torso = rx*rz*t

        # List with the states of the 26 joints of the robot
        #   joint_states[0]: names
        #   joint_states[1]: angles
        #   joint_states[2]: velocities
        #   joint_states[3]: efforts
        self.joint_states = [[0]*26]*4

        # Creating subscriber for the robot joints states
        self.create_subscription(msg_type=JointState, topic="/joint_states",
                                 callback=self.joints_states_callback,
                                 qos_profile=1)

        # Pointing and rest arms are chosen according to the object
        # position relative to the robot. While one arm points, the
        # other is in the rest configuration.
        self.pointing_arm = None
        self.rest_arm = None
        self.arm_rest_configuration = None

        # The head chain for the gaze communication.
        self.head = NaoRobot.TopCamera()

        # Arms and head rest configurations (robot standing)
        self.left_arm_rest = [1.4618600606918335,
                              0.16869807243347168,
                              -1.1796879768371582,
                              -0.40186595916748047,
                              0.11040592193603516]
        self.right_arm_rest = [1.4665460586547852,
                               -0.16878199577331543,
                               1.1826720237731934,
                               0.4019498825073242,
                               0.09353208541870117]
        self.head_rest = [0.0030260086059570312, -0.1595778465270996]

        # Publisher of joint commands
        self.joint_publisher = self.create_publisher(
            msg_type=JointAnglesWithSpeed,
            topic="/joint_angles", qos_profile=1)

        # Status of pointing and gaze
        self.pointing_status = -1
        self.gaze_status = -1

        # Pointing and gaze commands. First element indicates what
        # should to be indicated, second element is the desired speed of
        # the movement.
        self.point_to = [-1, -1]
        self.look_at = [-1, -1]

        # Objects for controllers of pointing and gaze communications
        self.solver = DQ_QuadprogSolver()
        self.controller_pointing = None
        self.controller_gaze = None

        # Creating LED groups for facial expressions
        self.facial_expression(expression="blank", duration=0,
                               create_groups=True)

        # Creating subscribers
        self.create_subscription(msg_type=ObjectIndication,
                                 topic="/pointing_command",
                                 callback=self.pointing_command_callback,
                                 qos_profile=QoSProfile(
                                     reliability=ReliabilityPolicy.RELIABLE,
                                     durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                     history=HistoryPolicy.KEEP_LAST,
                                     depth=50),
                                 callback_group=ReentrantCallbackGroup())
        self.create_subscription(msg_type=ObjectIndication,
                                 topic="/gaze_command",
                                 callback=self.gaze_command_callback,
                                 qos_profile=QoSProfile(
                                     reliability=ReliabilityPolicy.RELIABLE,
                                     durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                     history=HistoryPolicy.KEEP_LAST,
                                     depth=50),
                                 callback_group=ReentrantCallbackGroup())
        self.create_subscription(msg_type=String,
                                 topic="/nao_command",
                                 callback=self.nao_command_callback,
                                 qos_profile=1)
        self.create_subscription(msg_type=FacialExpression,
                                 topic="/expression_command",
                                 callback=self.expression_command_callback,
                                 qos_profile=1)

        # Publisher informing is robot is speaking or not
        self.speaking_publisher = self.create_publisher(
            msg_type=Bool,
            topic="/robot_speaking", qos_profile=1)

        # Timer to check if robot is speaking
        speaking_check_timer_period = 0.05
        self.speaking_check_timer = self.create_timer(
            speaking_check_timer_period, self.speaking_check_timer_callback,
            callback_group=MutuallyExclusiveCallbackGroup())

    def speaking_check_timer_callback(self):
        """
        Callback function for the timer to check if the robot is
        speaking or not. It publishes the result.

        """
        # Checking status using a client connecting to the robot server
        try:
            response = self.send_server_request("speech",
                                                response=True)

            # Reading output and publishing response
            msg = Bool()
            if response == '0':
                msg.data = False
            if response[0] == '1':
                msg.data = True

            self.speaking_publisher.publish(msg)

        except Exception:
            pass

    def send_server_request(self, command: str, response: bool = False):
        """
        Send a request to the robot server running externally.

        :param command: a String with the command to be sent
        :param response: a flag indicating if it should get a response or not
        :return: if it is the case, the message received from the server
        """

        s = socket.socket()
        s.connect((self.ip_server, self.port_server))
        s.send(bytes(command, 'utf-8'))
        if response:
            message = str(s.recv(1024).decode('utf-8'))
            s.close()
            return message
        s.close()

    def nao_command_callback(self, msg: String):
        """
        Callback function to get the command for one of the NAO's
        modules.

        :param msg: a std_msgs.msg/String message with the command
        :return:
        """
        self.send_server_request(msg.data)

    def markers_callback(self, msg: PoseStamped, id: int):
        """
        Callback function to get the poses of markers relative to
        marker zero.

        :param msg: a geometry_msgs/PoseStamped message with the pose
        :param id: the marker ID
        :return:
        """
        x = msg.pose.position.x
        y = msg.pose.position.y
        z = msg.pose.position.z
        w = msg.pose.orientation.w
        wx = msg.pose.orientation.x
        wy = msg.pose.orientation.y
        wz = msg.pose.orientation.z

        r = DQ([w, wx, wy, wz])
        p = DQ([0, x, y, z])

        self.markers_from_zero[id-1] = r + DQ.E*0.5*p*r

    def tracked_ids_callback(self, msg: IntArray):
        """
        Callback function to get the list of tracked markers IDs.

        :param msg: a package_with_interfaces/IntArray message with the
                    tracked markers IDs
        :return:
        """
        self.tracked_markers = msg.data

    def pointing_command_callback(self, msg: ObjectIndication):
        """
        Callback function to get the pointing command. If command is
        -1 or 0, arms go to rest configuration, otherwise the robot
        points to an object.

        :param msg: a package_with_interfaces/ObjectIndication message
                    with the object to be pointed to
        :return:
        """
        if self.point_to[0] != msg.object:
            # If it is a new command
            self.point_to[0] = msg.object
            self.point_to[1] = msg.speed

            # Start the loop in a new thread. If command is to go to
            # rest position, then the loop breaks after that. Otherwise,
            # it will end only when a class object is updated by a
            # call of this callback function with a new command. This
            # way, there are never more than one of this while loop
            # running at the same time.
            while self.point_to[0] == msg.object:
                try:
                    if self.point_to[0] == -1 or self.point_to[0] == 0:
                        # If command is -1 or 0, go to rest configuration

                        while True:
                            # Sending joint commands
                            self.publish_joint_commands(
                                NaoRobot.RightArm().get_joints_names(),
                                self.right_arm_rest, self.point_to[1])
                            self.publish_joint_commands(
                                NaoRobot.LeftArm().get_joints_names(),
                                self.left_arm_rest, self.point_to[1])

                            # Checking if it is close to the rest configuration
                            diff_right = []
                            diff_left = []
                            for i in range(0, len(self.right_arm_rest)):
                                diff_right.append(abs(self.right_arm_rest[i] -
                                                      self.get_chain_state(
                                                          NaoRobot.RightArm())[i]))
                                diff_left.append(abs(self.left_arm_rest[i] -
                                                     self.get_chain_state(
                                                         NaoRobot.LeftArm())[i]))

                            if all(x < 0.1 for x in diff_right + diff_left):
                                break

                        self.pointing_status = -1
                        break

                    else:
                        # If command is to point to an object, control
                        # the pointing line

                        if self.check_object_marker(self.point_to[0]):
                            # If object location is known

                            if self.point_to[0] != self.pointing_status:
                                # If it is a new command, update the
                                # pointing arm and some of the
                                # controller parameters

                                # Calculating object's pose relative to torso
                                x_torso_object = (self.x0_torso.conj() *
                                                  self.get_object_pose(
                                                      self.point_to[0]))

                                # Choose arm closer to the object
                                p0_object = x_torso_object.translation()
                                y_object = vec3(p0_object)[1]
                                if y_object >= 0:
                                    # If object is to the left or in
                                    # front of the robot torso, set left
                                    # arm as the pointing arm and right
                                    # arm should go to rest configuration
                                    self.pointing_arm = NaoRobot.LeftArm()
                                    self.rest_arm = NaoRobot.RightArm()
                                    self.arm_rest_configuration = (
                                        self.right_arm_rest)
                                else:
                                    # If object is to the right of the
                                    # robot torso, set right arm as the
                                    # pointing arm left arm should go to
                                    # rest configuration
                                    self.pointing_arm = NaoRobot.RightArm()
                                    self.rest_arm = NaoRobot.LeftArm()
                                    self.arm_rest_configuration = (
                                        self.left_arm_rest)

                                # Updating the controller
                                self.controller_pointing = (
                                    DQ_ClassicQPController(
                                        self.pointing_arm.kinematics(),
                                        self.solver))
                                self.controller_pointing.set_gain(700)
                                self.controller_pointing.set_damping(0.0001)
                                # The task variable to be controlled is
                                # a line attached to the robot arm
                                self.controller_pointing.set_control_objective(
                                    ControlObjective.Line)

                            self.pointing_gesture()
                except KeyboardInterrupt:
                    break

    def gaze_command_callback(self, msg: ObjectIndication):
        """
        Callback function to get the gaze command. If command is -1,
        head goes to rest configuration, if it is 0, robot looks to a
        face if there is one. Otherwise, the robot looks to an object.

        :param msg: a package_with_interfaces/ObjectIndication message
                    with the object to be looked at
        :return:
        """
        if self.look_at[0] != msg.object:
            # If it is a new command
            self.look_at[0] = msg.object
            self.look_at[1] = msg.speed

            # Start the loop in a new thread. If command is to go to
            # rest position or to look to a face, then the loop breaks
            # after that. Otherwise, it will end only when a class
            # object is updated by a call of this callback function
            # with a new command. This way, there are never more than
            # one of this while loop running at the same time.
            while self.look_at[0] == msg.object:
                try:
                    if self.look_at[0] == -1:
                        # If command is -1, stop face tracker and go to
                        # rest configuration

                        self.send_server_request("FaceTracker stopTracker")

                        while True:
                            # Sending joint command
                            self.publish_joint_commands(
                                NaoRobot.TopCamera().get_joints_names(),
                                self.head_rest, self.look_at[1])

                            # Checking if it is close to the rest configuration
                            diff = []
                            for i in range(0, len(self.head_rest)):
                                diff.append(abs(self.head_rest[i] -
                                                self.get_chain_state(
                                                    NaoRobot.TopCamera())[i]))

                            if all(x < 0.1 for x in diff):
                                break

                        self.gaze_status = -1
                        break

                    elif self.look_at[0] == 0:
                        # If command is 0, go back to rest configuration
                        # to make it easier to find a face, and if there
                        # is one, start tracking it

                        while True:
                            # Sending joint command
                            self.publish_joint_commands(
                                NaoRobot.TopCamera().get_joints_names(),
                                self.head_rest, self.look_at[1])

                            # Checking if it is close to the rest configuration
                            diff = []
                            for i in range(0, len(self.head_rest)):
                                diff.append(abs(self.head_rest[i] -
                                                self.get_chain_state(
                                                    NaoRobot.TopCamera())[i]))

                            if all(x < 0.1 for x in diff):
                                break

                        self.send_server_request("FaceTracker startTracker")

                        self.gaze_status = 0
                        break

                    else:
                        # If command is to look at an object, control
                        # the gaze line

                        if self.check_object_marker(self.look_at[0]):
                            # If object location is known

                            if self.look_at[0] != self.gaze_status:
                                # If it is a new command, stop face
                                # tracker and update some of the controller
                                # parameters

                                self.send_server_request("FaceTracker stopTracker")

                                # Updating the controller
                                self.controller_gaze = DQ_ClassicQPController(
                                    self.head.kinematics(), self.solver)
                                self.controller_gaze.set_gain(900)
                                self.controller_gaze.set_damping(0.01)
                                # The task variable to be controlled is
                                # a line attached to the robot head
                                self.controller_gaze.set_control_objective(
                                    ControlObjective.Line)

                            self.gaze()
                except KeyboardInterrupt:
                    break

    def expression_command_callback(self, msg: FacialExpression):
        """
        Callback function to get the command for a facial expression.

        :param msg: a package_with_interfaces/FacialExpression message
                    with the command
        :return:
        """
        self.facial_expression(expression=msg.name, duration=msg.duration)
        self.get_logger().info(f"Expression {msg.name} for {msg.duration}s")

    def joints_states_callback(self, msg: JointState):
        """
        Callback function to get the robot joints states.

        :param msg: a sensor_msgs/JointStates message with the joint states
        :return:
        """
        self.joint_states[0] = msg.name
        self.joint_states[1] = msg.position
        self.joint_states[2] = msg.velocity
        self.joint_states[3] = msg.effort

    def publish_joint_commands(self, joint_names: [str], joint_angles: [float],
                               speed: float = 0.5, relative: int = 0):
        """
        Publish message to /joint_angles topic to send a command to the
        robot joints.

        :param joint_names: a list with the joint names
        :param joint_angles: a list with the desired joint angles
        :param speed: desired fraction of maximum joint velocity
        :param relative: if absolute (0-default) or relative (1) angle
        :return:
        """
        msg = JointAnglesWithSpeed()
        msg.joint_names = joint_names
        msg.joint_angles = joint_angles
        msg.speed = speed
        msg.relative = relative
        self.joint_publisher.publish(msg)

    def get_chain_state(self, chain: NaoRobot) -> [float]:
        """
        Get the state of the joints in a chain of the robot.

        :param chain: an object of the chain class
        :return: a list with the chain joint states
        """
        chain_state = []
        for joint in chain.get_joints_names():
            if self.joint_states[0][0] != 0:
                index = self.joint_states[0].index(joint)
                chain_state.append(self.joint_states[1][index])

        return chain_state

    def get_object_pose(self, object: int):
        """
        Obtain the object pose relative to marker 0.

        :param object: the object ID
        :return:
        """
        # Looking for object ID in the objects list
        for i in range(0, len(self.objects)):
            if self.objects[i][0] == object:
                break

        # Translation from marker to the object
        t = 1 + DQ.E*0.5*DQ(self.objects[i][2])

        return self.markers_from_zero[self.objects[i][1]-1] * t

    def check_object_marker(self, object: int):
        """
        Check if the marker related to an object is being detected.

        :param object: the object ID
        :return: True if detected, False if not
        """
        # Obtaining the list index of the object of interest
        found = False
        for i in range(0, len(self.objects)):
            if self.objects[i][0] == object:
                found = True
                break

        if found:
            if self.objects[i][1] in self.tracked_markers:
                return True
            else:
                message = (f"Unknown location of object {object} "
                           f"(marker {self.objects[i][1]}).")
                self.get_logger().info("\033[31m{}\033[0m".format(message))
                return False
        else:
            message = (f"Object {object} is unknown.")
            self.get_logger().info("\033[31m{}\033[0m".format(message))
            return False

    def pointing_gesture(self):
        """
        Update the robot's pointing gesture communication. It implements
        a line-to-line controller with inequality constraints. The
        control objective is to align a line attached to the robot arm
        with a reference line connected to the object. The inequality
        constraints account for the joint limits.

        :return:
        """
        self.pointing_status = self.point_to[0]

        # Current configuration
        q = self.get_chain_state(self.pointing_arm)

        # Calculating object's pose relative to torso
        x_torso_object = (self.x0_torso.conj() *
                          self.get_object_pose(self.point_to[0]))

        # Reference line from a point in the arm (frame 3) to the object
        arm_point = self.pointing_arm.kinematics().fkm(q, 1).translation()
        direction = normalize(x_torso_object.translation() - arm_point)
        point = x_torso_object.translation()
        line_ref = direction + DQ.E*cross(point, direction)

        # Line attached to the robot, created using elbow and hand
        point1 = self.pointing_arm.kinematics().fkm(q, 3).translation()
        point2 = self.pointing_arm.kinematics().fkm(q).translation()
        direction = normalize(point2 - point1)
        line = direction + DQ.E*(cross(point2, direction))

        x = self.pointing_arm.kinematics().fkm(q)
        line_effector = x.conj()*line*x  # line relative to end effector

        # Setting line attached to the robot to be controlled
        self.controller_pointing.set_primitive_to_effector(line_effector)

        # Getting joints limits
        limits = self.pointing_arm.get_joints_limits()
        n_joints = int(len(limits)/2)
        lower = np.array(limits[0:n_joints]).reshape(n_joints, 1)
        upper = np.array(limits[n_joints:2*n_joints]).reshape(n_joints, 1)
        q_array = np.array(q).reshape(n_joints, 1)
        error_upper = q_array - upper
        error_lower = q_array - lower

        # Setting inequality constraints
        eta_constraints = 500.0
        A = np.row_stack([np.identity(n_joints), -np.identity(n_joints)])
        b = np.row_stack([-eta_constraints*error_upper,
                          eta_constraints*error_lower])
        self.controller_pointing.set_inequality_constraint(A, b)

        # Calculating control signal (joints velocities)
        u = self.controller_pointing.compute_setpoint_control_signal(
            q, vec8(line_ref))

        # Integrating to get the joints configuration
        T = 0.001  # integration step
        q = q + T*u

        # Sending joints commands to the robot
        joint_angles = []
        for j in range(0, n_joints):
            joint_angles.append(q[j])
        self.publish_joint_commands(self.pointing_arm.get_joints_names(),
                                    joint_angles, self.point_to[1])
        self.publish_joint_commands(self.rest_arm.get_joints_names(),
                                    self.arm_rest_configuration,
                                    self.point_to[1])

        self.get_logger().info(f"pointing line error: "
                               f"{np.linalg.norm(vec8(line_ref - line))}")

    def gaze(self):
        """
        Update the robot's gaze communication. It implements a
        line-to-line controller with inequality constraints. The control
        objective is to align a line attached to the robot head with a
        reference line connected to the object. The inequality
        constraints account for the joint limits.

        :return:
        """
        self.gaze_status = self.look_at[0]

        # Current configuration
        q = self.get_chain_state(self.head)

        # Calculating object's pose relative to torso
        x_torso_object = (self.x0_torso.conj() *
                          self.get_object_pose(self.look_at[0]))

        # Reference line from a point above the neck and behind the
        # top camera to the object
        head = self.head.kinematics().fkm(q, 0).translation()
        head = head + DQ([0, 0, self.head.top_camera_z])
        direction = normalize(x_torso_object.translation() - head)
        point = x_torso_object.translation()
        line_ref = direction + DQ.E * cross(point, direction)

        # Line attached to the robot, aligned with the x-axis of the top
        # camera frame
        r = self.head.kinematics().fkm(q).rotation()
        direction = r * DQ.i * r.conj()
        point = self.head.kinematics().fkm(q).translation()
        line = direction + DQ.E * (cross(point, direction))

        x = self.head.kinematics().fkm(q)
        line_effector = x.conj()*line*x  # line relative to end effector

        # Setting line attached to the robot to be controlled
        self.controller_gaze.set_primitive_to_effector(line_effector)

        # Getting joints limits
        limits = self.head.get_joints_limits()
        n_joints = int(len(limits)/2)
        lower = np.array(limits[0:n_joints]).reshape(n_joints, 1)
        upper = np.array(limits[n_joints:2*n_joints]).reshape(n_joints, 1)
        q_array = np.array(q).reshape(n_joints, 1)
        error_upper = q_array - upper
        error_lower = q_array - lower

        # Setting inequality constraints
        eta_constraints = 900.0
        A = np.row_stack([np.identity(n_joints), -np.identity(n_joints)])
        b = np.row_stack([-eta_constraints*error_upper,
                          eta_constraints*error_lower])
        self.controller_gaze.set_inequality_constraint(A, b)

        # Calculating control signal (joints velocities)
        u = self.controller_gaze.compute_setpoint_control_signal(
            q, vec8(line_ref))

        # Integrating to get the joints configuration
        T = 0.001  # integration step
        q = q + T*u

        # Sending joints commands to the robot
        joint_angles = []
        for j in range(0, n_joints):
            joint_angles.append(q[j])
        self.publish_joint_commands(self.head.get_joints_names(),
                                    joint_angles, self.look_at[1])

        self.get_logger().info(f"gaze line error: "
                               f"{np.linalg.norm(vec8(line_ref - line))}")

    def facial_expression(self, expression: str, duration: float,
                          create_groups: bool = False):
        """
        Make the robot facial expressions using the LEDs in its eyes.
        It creates a client to send a request to the robot server, which
        will execute the expression command.

        The available facial expressions use personalised LED groups
        in the robot, so it is necessary to create them. Once they
        are created, they will remain in the robot's memory until it's
        restarted, so there is no need to do it every time an
        expression is to be executed.

        :param expression: the name of the facial expression
        :param duration: the duration of the facial expression
        :param create_groups: create LED groups (True) or not (False)
        :return:
        """
        if create_groups:
            # Create LED groups
            self.send_server_request("expression groups")

        if expression == "blank":
            # Turn the eyes' LEDs off
            self.send_server_request("expression off")
        else:
            # Execute the desired expression
            command = "expression " + expression + " " + str(duration)
            self.send_server_request(command)


def main(args=None):
    rclpy.init(args=args)

    rc = RobotCommunicationNode()
    rc.send_server_request('Motion wakeUp')
    rc.send_server_request('RobotPosture goToPosture Stand 0.5')
    rc.send_server_request("FaceTracker stopTracker")

    executor = MultiThreadedExecutor()
    executor.add_node(rc)

    try:
        executor.spin()
    except KeyboardInterrupt:
        rc.robot_server.kill()
        rc.destroy_node()
        rclpy.shutdown()
        pass
    except Exception as e:
        print(e)

    rc.robot_server.kill()
    rc.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
