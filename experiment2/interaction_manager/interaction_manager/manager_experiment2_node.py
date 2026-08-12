import time
import random
from datetime import datetime
import subprocess
import socket
from socket import error as socket_error

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, ReliabilityPolicy, HistoryPolicy,
                       DurabilityPolicy)
from ament_index_python.packages import get_package_share_directory
from std_msgs.msg import String, Int64, Bool, Int16MultiArray, Float64
from geometry_msgs.msg import PoseStamped
from dqrobotics import *

from package_with_interfaces.msg import (ObjectIndication, NaoCommand,
                                         FacialExpression, RobotSpeech)


class InteractionManagerNode(Node):

    def __init__(self):
        super().__init__('manager_experiment2_node')
        # Getting parameters from launch file
        self.get_launch_parameters()

        # Creating log file
        if self.logging:
            path = get_package_share_directory("interaction_manager")
            path = path.split("install")[0] + "src/interaction_manager/log/"
            name = datetime.now().strftime("%d-%m-%Y_%H.%M.%S") + '.log'
            self.log_file = open(path + name, 'w')

        # Current status of the interaction
        # 0: not started yet
        # 1: instructions and practice round
        # 2: practice finished, task not started yet
        # 3: task
        # 4: task finished, waiting for questionnaire
        # 5: final messages if last round
        self.interaction_status = 0

        # Flag indicating if robot is speaking (True) or not (False)
        self.robot_speaking = False

        # Variable to save the start time of the indication
        self.time_indicating = 0

        # Number of errors in each phase
        self.errors_phase1 = 0
        self.errors_phase2 = 0

        # Array with the poses of markers
        self.max_markers = 50
        self.markers_poses = [DQ([1])]*(self.max_markers-1)

        # Random order of conditions
        # 1: easy-EX
        # 2: easy-EXIM
        # 3: difficult-EX
        # 4: difficult-EXIM
        self.conditions_order = [1, 2, 3, 4]
        random.shuffle(self.conditions_order)

        if self.logging:
            message = f"Order of conditions: {self.conditions_order}\n\n"
            self.log_file.write(message)

            count = 1
            for g in self.groups:
                message = f"Group {count}: main object {g[0]}\n         "
                message = message + "colour: " + g[2] + "\n         "
                message = message + "objects: " + str(g[1]) + "\n"

                self.log_file.write(message)
                count = count + 1
            self.log_file.write("\n")

        # Current sequence information:
        self.sequence = [0]
        self.sequence_length = 1

        # Flag indicating command to show sequence
        self.show_sequence = False

        # Task score
        # +1 for each correct object added
        # -1 for each wrong object added
        # -2 for each time that sequence is shown (except first one)
        self.score = 0

        # Instructions step
        self.instructions_step = 0

        # Creating ROS2 interfaces
        self.create_interfaces()

    def get_launch_parameters(self):
        """
        Get parameters from the launch file.

        :return:
        """
        # Parameter defining if log files should be created or not.
        # If not defined, default is True
        self.declare_parameter("log", rclpy.Parameter.Type.BOOL)
        try:
            self.logging: bool = self.get_parameter(
                'log').get_parameter_value().bool_value
        except:
            self.logging = True

        # Parameter defining if video should be recorded (True) or not (False).
        # If not defined, default is True
        self.declare_parameter("record", rclpy.Parameter.Type.BOOL)
        try:
            self.record: bool = self.get_parameter(
                'record').get_parameter_value().bool_value
        except:
            self.record = True

        # Parameter defining volume of the robot
        self.declare_parameter("volume", rclpy.Parameter.Type.INTEGER)
        self.volume: bool = self.get_parameter(
            'volume').get_parameter_value().integer_value

        # IP address and port to communicate with robot server.
        # If not defined, defaults are '127.0.0.1' and 1111, respectively.
        self.declare_parameter("ip_robot_server", rclpy.Parameter.Type.STRING)
        try:
            self.ip_robot_server: str = self.get_parameter(
                'ip_robot_server').get_parameter_value().string_value
        except:
            self.ip_robot_server = '127.0.0.1'
        self.declare_parameter("port_robot_server",
                               rclpy.Parameter.Type.INTEGER)
        try:
            self.port_robot_server: int = (self.get_parameter('port_robot_server').
                                     get_parameter_value().integer_value)
        except:
            self.port_robot_server = 1111

        # IP address and port to communicate with record server.
        # If not defined, defaults are '192.168.0.103' and 2222, respectively.
        self.declare_parameter("ip_record", rclpy.Parameter.Type.STRING)
        try:
            self.ip_record: str = (self.get_parameter('ip_record').
                                   get_parameter_value().string_value)
        except:
            self.ip_record = '192.168.0.103'
        self.declare_parameter("port_record", rclpy.Parameter.Type.INTEGER)
        try:
            self.port_record: int = (self.get_parameter('port_record').
                                     get_parameter_value().integer_value)
        except:
            self.port_record = 2222

        # Defining the assembly area.
        self.declare_parameter("assembly_area",
                               rclpy.Parameter.Type.DOUBLE_ARRAY)
        self.assembly_area = self.get_parameter(
            "assembly_area").get_parameter_value().double_array_value

        # List of objects and their associated markers
        # [[object1_ID, object1_marker],
        #  [object2_ID, object2_marker],
        #  ... ]
        self.declare_parameter("objects", rclpy.Parameter.Type.INTEGER_ARRAY)
        list_objects = self.get_parameter(
            "objects").get_parameter_value().integer_array_value
        i = 0
        self.objects = []
        while i < len(list_objects):
            self.objects.append([list_objects[i], list_objects[i+1]])
            i = i + 2

        # List of groups of objects
        # Each group has one main object that represents it, so if the
        # robot needs to indicate the group, for example, it can
        # indicate this main object. There is also a color associated
        # to the group and a list of objects that belong to it.
        self.groups = []
        for i in range(1, 11):
            try:
                name = "group" + str(i)
                self.declare_parameter(name,
                                       rclpy.Parameter.Type.INTEGER_ARRAY)
                group = self.get_parameter(
                    name).get_parameter_value().integer_array_value
                self.groups.append([group[0], list(group[1:len(group)])])
                # [[group1_object, [objects_in_group1]],
                #  [group2_object, [objects_in_group2]],
                #  ... ]
            except:
                if i == 2:
                    message = f"{i - 1} group defined"
                    self.get_logger().info("\033[32m{}\033[0m".format(message))
                else:
                    message = f"{i - 1} groups defined"
                    self.get_logger().info("\033[32m{}\033[0m".format(message))

                break

        # Name of the colors of each group
        self.declare_parameter("colors", rclpy.Parameter.Type.STRING_ARRAY)
        colors = self.get_parameter(
            "colors").get_parameter_value().string_array_value

        # Adding color name to the self.groups argument.
        for i in range(0, len(self.groups)):
            # [[group1_object, [objects_in_group1], group1_color],
            #  [group2_object, [objects_in_group2], group2_color],
            #  ... ]
            self.groups[i].append(colors[i])

        # Sequence of groups to indicate in phase 1:
        self.declare_parameter("sequence1", rclpy.Parameter.Type.INTEGER_ARRAY)
        self.sequence_phase1 = list(self.get_parameter(
            "sequence1").get_parameter_value().integer_array_value)

    def create_interfaces(self):
        """
        Create publishers, subscribers, and services to communicate with
        other nodes.

        """
        # Publisher for speech commands:
        self.voice_command_publisher = self.create_publisher(msg_type=String,
                                                             topic='/speech',
                                                             qos_profile=1)

        # Publisher for pointing commands:
        self.pointing_command_publisher = self.create_publisher(
            msg_type=ObjectIndication, topic='/pointing_command',
            qos_profile=QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                                   durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                   history=HistoryPolicy.KEEP_LAST, depth=50))

        # Publisher for gaze commands:
        self.gaze_command_publisher = self.create_publisher(
            msg_type=ObjectIndication, topic='/gaze_command',
            qos_profile=QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                                   durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                   history=HistoryPolicy.KEEP_LAST, depth=50))

        # Publisher for facial expressions commands:
        self.expressions_command_publisher = self.create_publisher(
            msg_type=FacialExpression, topic='/expression_command',
            qos_profile=QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                                   durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                   history=HistoryPolicy.KEEP_LAST, depth=50))

        # Publisher for interaction status:
        self.interaction_status_publisher = self.create_publisher(
            msg_type=Int64, topic="/interaction_status",
            qos_profile=QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                                   durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                   history=HistoryPolicy.KEEP_LAST, depth=50))

        # Publisher for robot speech monitoring:
        self.monitor_speech_publisher = self.create_publisher(
            msg_type=RobotSpeech, topic="/monitor_speech",
            qos_profile=QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                                   durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                   history=HistoryPolicy.KEEP_LAST, depth=50))

        # Publisher to set buttons states in the screen application:
        self.button_states_publisher = self.create_publisher(
            msg_type=Bool, topic="/button_state",
            qos_profile=QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                                   durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                   history=HistoryPolicy.KEEP_LAST, depth=50))

        # Publisher to set the colour sequence:
        self.sequence_publisher = self.create_publisher(
            msg_type=Int16MultiArray, topic="/sequence",
            qos_profile=QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                                   durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                   history=HistoryPolicy.KEEP_LAST, depth=50))

        # Publisher to set for how long the sequence should be displayed:
        self.sequence_time_publisher = self.create_publisher(
            msg_type=Float64, topic="/sequence_time",
            qos_profile=QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                                   durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                   history=HistoryPolicy.KEEP_LAST, depth=50))

        # Publisher to indicate an added object:
        self.added_publisher = self.create_publisher(
            msg_type=Int64, topic="/added",
            qos_profile=QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                                   durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                   history=HistoryPolicy.KEEP_LAST, depth=50))

        # Publisher to show the colour sequence:
        self.show_sequence_publisher = self.create_publisher(
            msg_type=Float64, topic="/show_sequence",
            qos_profile=QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                                   durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                   history=HistoryPolicy.KEEP_LAST, depth=50))

        # Publisher to indicate what should be displayed in second screen:
        self.screen2_publisher = self.create_publisher(
            msg_type=Int64, topic="/screen2",
            qos_profile=QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                                   durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                   history=HistoryPolicy.KEEP_LAST, depth=50))

        # Subscriber for the interaction status:
        self.create_subscription(msg_type=Int64,
                                 topic="/interaction_status",
                                 callback=self.interaction_status_callback,
                                 qos_profile=1)

        # Subscribers for markers poses relative to marker 0:
        for i in range(1, self.max_markers):
            topic_name = "/x0_" + str(i)
            self.create_subscription(
                msg_type=PoseStamped,
                topic=topic_name,
                callback=lambda msg, id=i: self.markers_callback(msg, id),
                qos_profile=1)

        # Subscriber to monitor if robot is speaking or not:
        self.create_subscription(msg_type=Bool,
                                 topic="/robot_speaking",
                                 callback=self.robot_speaking_msg_callback,
                                 qos_profile=10)

        # Subscriber for clicks in Next button during instructions:
        self.create_subscription(msg_type=Int64,
                                 topic="/next_instructions_click",
                                 callback=self.next_instructions_callback,
                                 qos_profile=10)

        # Subscriber for clicks in Show sequence button:
        self.create_subscription(msg_type=Bool,
                                 topic="/show_sequence_click",
                                 callback=self.show_sequence_click_callback,
                                 qos_profile=10)

    def configure_condition(self, condition: int,
                            easy_len: int = 4, easy_time: int = 4,
                            difficult_len: int = 8, difficult_time: int = 2):
        """
        Configuring the task condition. Sets the length of the colour
        sequence and the amount of time it will be displayed on the
        screen.

        :param condition: the current condition
        :param easy_len: sequence length for easy conditions
        :param easy_time: display duration for easy conditions (in s)
        :param difficult_len: sequence length for difficult conditions
        :param difficult_time: display duration for difficult conditions (in s)
        :return:
        """
        # Defining length of sequence and display duration according to
        # the condition
        if condition == 1 or condition == 2:
            # Conditions easy-EX and easy-EXIM
            self.sequence_length = easy_len
            self.sequence_time = easy_time
        if condition == 3 or condition == 4:
            # Conditions difficult-EX and difficult-EXIM
            self.sequence_length = difficult_len
            self.sequence_time = difficult_time

        self.sequence = random.sample([17, 18, 19], counts=[5, 5, 5],
                                      k=self.sequence_length)

        # Sending information to screen application node
        msg = Int16MultiArray()
        msg.data = self.sequence
        self.sequence_publisher.publish(msg)
        msg = Float64()
        msg.data = float(easy_time)
        self.sequence_time_publisher.publish(msg)

    def interaction_status_callback(self, msg: Int64):
        """
        Callback function to get information about changes in the
        status of the interaction.

        :param msg: an integer indicating the current stage
        :return:
        """
        self.interaction_status = msg.data

    def next_instructions_callback(self, msg: Int64):
        """
        Callback function for the click of the "Next" button
        in the screen application during instructions phase.

        :param msg: an integer indicating to go forward
        :return:
        """
        self.instructions_step = self.instructions_step + msg.data
        if self.instructions_step > 6:
            self.instructions_step = 6

    def show_sequence_click_callback(self, msg: Bool):
        """
        Callback function for the click of the "Show sequence" button
        in the screen application.

        :param msg: a boolean indicating the command
        :return:
        """
        self.show_sequence = msg.data

    def robot_speaking_msg_callback(self, msg: Bool):
        """
        Callback function to get the information from the topic
        indicating if the robot is speaking.

        :param msg: a boolean indicating if robot is speaking or not
        """
        # Updating flag indicating if robot is speaking or not
        self.robot_speaking = msg.data

    def markers_callback(self, msg: PoseStamped, marker_id: int):
        """
        Callback function to get the pose of markers relative to
        marker zero.

        :param msg: a geometry_msgs/PoseStamped message with the pose
        :param marker_id: the marker ID
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

        self.markers_poses[marker_id - 1] = r + DQ.E*0.5*p*r

    def speech_routine(self, text: str):
        """
        Execute the routine for a robot speech. It sends the command,
        updates the screen application accordingly.

        :param text: the text that the robot should speak
        :return:
        """
        # Setting robot volume
        s = socket.socket()
        s.connect((self.ip_robot_server, self.port_robot_server))
        command = "ALAudioDevice setOutputVolume " + str(self.volume)
        s.send(bytes(command, 'utf-8'))
        s.close()

        # Send speech command to the robot
        msg = String()
        msg.data = text
        self.voice_command_publisher.publish(msg)

        # Wait until robot starts speaking
        start = time.time()
        while not self.robot_speaking:
            try:
                #self.get_logger().info("...Waiting for speech to start...")

                if time.time() - start > 3:
                    self.voice_command_publisher.publish(msg)
                    start = time.time()

                rclpy.spin_once(self)
            except KeyboardInterrupt:
                break

        if self.logging:
            self.write_log("SPEECH", text)

        # Update screen application
        msg = RobotSpeech()
        msg.status = True
        msg.text = text
        self.monitor_speech_publisher.publish(msg)

        # Wait until robot stops speaking
        while self.robot_speaking:
            try:
                #self.get_logger().info("...Waiting for speech to finish...")

                rclpy.spin_once(self)
            except KeyboardInterrupt:
                break

        # Update screen application
        msg = RobotSpeech()
        msg.status = False
        msg.text = text
        self.monitor_speech_publisher.publish(msg)

        # Setting robot volume to zero
        s = socket.socket()
        s.connect((self.ip_robot_server, self.port_robot_server))
        command = "ALAudioDevice setOutputVolume 0"
        s.send(bytes(command, 'utf-8'))
        s.close()

    def is_inside(self, object_id: int):
        """
        Check if an object is inside the assembly area or not.

        :param object_id: the object ID
        :return: True if object is inside and False if not
        """

        # Defining assembly area corners frames
        if len(self.assembly_area) == 4:
            # Each corner is defined by a marker
            tl_pose = self.markers_poses[int(self.assembly_area[0]) - 1]
            tr_pose = self.markers_poses[int(self.assembly_area[1]) - 1]
            bl_pose = self.markers_poses[int(self.assembly_area[2]) - 1]
            br_pose = self.markers_poses[int(self.assembly_area[3]) - 1]

        if len(self.assembly_area) == 13:
            # Each corner is at a given location relative to a single marker
            marker_pose = self.markers_poses[int(self.assembly_area[0]) - 1]
            rotation = normalize(DQ([self.assembly_area[1],
                                     self.assembly_area[2],
                                     self.assembly_area[3],
                                     self.assembly_area[4]]))
            tl_pose = marker_pose*(1 + DQ.E*0.5*DQ([self.assembly_area[5],
                                                    self.assembly_area[6],
                                                    0.0]))*rotation
            tr_pose = marker_pose*(1 + DQ.E*0.5*DQ([self.assembly_area[7],
                                                    self.assembly_area[8],
                                                    0.0]))*rotation
            bl_pose = marker_pose*(1 + DQ.E*0.5*DQ([self.assembly_area[9],
                                                    self.assembly_area[10],
                                                    0.0]))*rotation
            br_pose = marker_pose*(1 + DQ.E*0.5*DQ([self.assembly_area[11],
                                                    self.assembly_area[12],
                                                    0.0]))*rotation

        # Getting the marker associated to the object
        obj_marker = None
        for o in self.objects:
            if o[0] == object_id:
                obj_marker = o[1]
                break

        if obj_marker is None:
            return self.get_logger().info(
                f'Object {object_id} not defined.')

        # Checking top left corner of assembly area
        pose = tl_pose.conj() * self.markers_poses[obj_marker-1]
        position = pose.translation().vec3()
        if position[0] < 0 or position[1] > 0:
            return False

        # Checking top right corner of assembly area
        pose = tr_pose.conj() * self.markers_poses[obj_marker-1]
        position = pose.translation().vec3()
        if position[0] > 0 or position[1] > 0:
            return False

        # Checking bottom left corner of assembly area
        pose = bl_pose.conj() * self.markers_poses[obj_marker-1]
        position = pose.translation().vec3()
        if position[0] < 0 or position[1] < 0:
            return False

        # Checking bottom right corner of assembly area
        pose = br_pose.conj() * self.markers_poses[obj_marker-1]
        position = pose.translation().vec3()
        if position[0] > 0 or position[1] < 0:
            return False

        return True

    def check_inside(self):
        """
        Check which objects are inside the assembly area.

        :return: a list with the IDs of the objects inside the area
        """
        inside = []
        for obj in self.objects:
            if self.is_inside(obj[0]):
                inside.append(obj[0])

        return inside

    def write_log(self, tag: str, message=""):
        """
        Write information in the log file.

        :param tag: tag identifying the input in the log file
        :param message: message to be written in the log file
        :return:
        """
        # Writing a line in log file with the following format:
        #      dd.mm.yy_HH:MM:SS [tag] message
        if self.logging:
            self.log_file.write(datetime.now().strftime("%d.%m.%Y_%H:%M:%S"))
            self.log_file.write(" [" + tag + "] " + str(message) + "\n")


def main(args=None):

    try:
        rclpy.init(args=args)

        im = InteractionManagerNode()
        last = im.interaction_status  # last interaction status
        last_instruction = im.instructions_step  # last instruction step

        start_time = 0  # start of a round
        between_time = 0  # start of time between objects

        next_object = 0  # index of the next object to be added
        object_inside = False  # flag to indicate if there is object inside
        condition = 0  # index of the current condition
        errors = 0  # number of errors in the current round
        looking = False  # flag to indicate if robot is looking to object

        record_subprocess = None

        # Waiting a few seconds before start publishing
        time.sleep(2)

        # Configuring practice round
        im.sequence = [17, 18, 19]
        im.sequence_time = 3
        msg = Int16MultiArray()
        msg.data = im.sequence
        im.sequence_publisher.publish(msg)
        msg = Float64()
        msg.data = float(im.sequence_time)
        im.sequence_time_publisher.publish(msg)

        # Task score
        im.score = 0

        while rclpy.ok():
            # Start instructions and practice
            if im.interaction_status == 1:
                if im.interaction_status != last:
                    last = im.interaction_status

                    # Start recording
                    if im.record:
                        # Recording video from camera used for markers
                        path = get_package_share_directory("interaction_manager")
                        path = (path.split("install")[0] +
                                "src/interaction_manager/videos/")
                        name = (datetime.now().strftime("%d-%m-%Y_%H.%M.%S_top") +
                                '.mp4')
                        command = ["rosimg2mp4",
                                   "-i", "/camera_markers_edited",
                                   "-o", path+name,
                                   "-fs", "1920", "1080"]
                        record_subprocess = subprocess.Popen(command)

                        # Starting video and audio recording through
                        # record server
                        s = socket.socket()
                        s.settimeout(4)
                        try:
                            s.connect((im.ip_record, im.port_record))
                            s.send(bytes('start audio video', 'utf-8'))
                            s.close()
                        except socket.error:
                            pass

                    text = ("You will see a sequence of colours/symbols shown "
                            "on this screen and will be asked to memorise it "
                            "as best as you can. You will then replicate the "
                            "sequence with the objects in front of you. The "
                            "blocks are divided in groups, and each "
                            "group represents a colour/symbol. When you are "
                            "ready, click the button on top of this screen to "
                            "see the sequence, and then click Next to "
                            "continue with the instructions. Do not move the "
                            "blocks yet.")
                    msg = RobotSpeech()
                    msg.status = False
                    msg.text = text
                    im.monitor_speech_publisher.publish(msg)
                else:
                    # If there is a command to show the sequence
                    if im.show_sequence:
                        msg = Float64()
                        msg.data = float(im.sequence_time)
                        im.show_sequence_publisher.publish(msg)

                        # Waiting while sequence is shown
                        time.sleep(im.sequence_time)

                        im.show_sequence = False

                    if im.instructions_step == 0:
                        if im.instructions_step != last_instruction:
                            last_instruction = im.instructions_step
                            text = ("You will see a sequence of "
                                    "colours/symbols shown on this screen and "
                                    "will be asked to memorise it as best as "
                                    "you can. You will then replicate the "
                                    "sequence with the objects in front of "
                                    "you. The blocks are divided in groups, "
                                    "and each group represents a "
                                    "colour/symbol. When you are ready, click "
                                    "the button on top of this screen to see "
                                    "the sequence, and then click Next to "
                                    "continue with the instructions. Do NOT "
                                    "move the blocks yet.")
                            msg = RobotSpeech()
                            msg.status = False
                            msg.text = text
                            im.monitor_speech_publisher.publish(msg)
                    if im.instructions_step == 1:
                        if im.instructions_step != last_instruction:
                            last_instruction = im.instructions_step

                            text = ("The first object should be from the "
                                    "magenta/□ group. Put ONE of these in the "
                                    "designated area in front of you and "
                                    "click Next to continue.")
                            msg = RobotSpeech()
                            msg.status = False
                            msg.text = text
                            im.monitor_speech_publisher.publish(msg)

                        # Checking object added
                        # If from correct group, update screen
                        # If not, does nothing
                        new = im.check_inside()
                        if new:
                            object_inside = True
                            for g in im.groups:
                                if g[0] == im.sequence[next_object]:
                                    if new[0] in g[1]:
                                        # If new object is from right group
                                        msg = Int64()
                                        msg.data = g[0]
                                        im.added_publisher.publish(msg)

                                        while im.instructions_step == 1:
                                            rclpy.spin_once(im)

                                        break

                    if im.instructions_step == 2:
                        # Checking object added
                        # If correct group, update screen and continue
                        # If not, goes back to previous instruction state
                        new = im.check_inside()
                        if new:
                            for g in im.groups:
                                if g[0] == im.sequence[next_object]:
                                    if new[0] in g[1]:
                                        # If new object is from right group
                                        text = (
                                            "Good. This is correct. And the "
                                            "screen was updated, as you can "
                                            "see. Now put the block back and "
                                            "click Next for more instructions.")
                                        msg = RobotSpeech()
                                        msg.status = False
                                        msg.text = text
                                        im.monitor_speech_publisher.publish(msg)

                                        # Waiting for object to be removed
                                        while im.check_inside():
                                            im.instructions_step = 2
                                            im.show_sequence = False
                                            rclpy.spin_once(im)

                                        object_inside = False

                                        # Update screen after object removed
                                        msg = Int64()
                                        msg.data = 0
                                        im.added_publisher.publish(msg)

                                        # Waiting for click in the button
                                        while im.instructions_step == 2:
                                            im.show_sequence = False
                                            rclpy.spin_once(im)

                                        next_object = next_object + 1

                                    else:
                                        # If new object is from wrong group
                                        im.instructions_step = 1
                                    break
                        else:
                            # If there is no new object
                            im.instructions_step = 1

                            object_inside = False

                    if im.instructions_step == 3:
                        if im.instructions_step != last_instruction:
                            last_instruction = im.instructions_step

                            text = ("Now, if you remember correctly, the next "
                                    "block should be from the green/△ group "
                                    "(You can click the button to see the "
                                    "sequence again if you need). \n\n"
                                    "However, as this is a practice, put a "
                                    "block from the WRONG group in the "
                                    "designated area and click Next to see "
                                    "what happens.")
                            msg = RobotSpeech()
                            msg.status = False
                            msg.text = text
                            im.monitor_speech_publisher.publish(msg)

                        # Checking object added
                        # If correct group, does nothing
                        # If not, update screen
                        new = im.check_inside()
                        if new:
                            object_inside = True
                            for g in im.groups:
                                if new[0] in g[1]:
                                    if g[0] != im.sequence[next_object]:
                                        # If new object is from wrong group
                                        msg = Int64()
                                        msg.data = g[0]
                                        im.added_publisher.publish(msg)

                                        # Waiting for click in the button
                                        while im.instructions_step == 3:
                                            im.show_sequence = False
                                            rclpy.spin_once(im)
                                    else:
                                        im.instructions_step = 3

                                    break

                                im.instructions_step = 3
                        else:
                            # If there is no new object
                            im.instructions_step = 3

                            object_inside = False

                    if im.instructions_step == 4:
                        # Checking object added
                        # If wrong group, update screen and continue
                        # If not, goes back to previous instruction state
                        if im.instructions_step != last_instruction:
                            last_instruction = im.instructions_step
                            new = im.check_inside()
                            if new:
                                object_inside = True
                                for g in im.groups:
                                    if new[0] in g[1]:
                                        if g[0] != im.sequence[next_object]:
                                            # If new object is from wrong group
                                            text = (
                                                "This is indeed a block from "
                                                "the wrong group. The "
                                                "sequence shown below was not "
                                                "updated, as you can see. Put "
                                                "the block back and you can "
                                                "now finish the sequence with "
                                                "the correct blocks. Remember "
                                                "to add only one at a time, "
                                                "and click Next when you "
                                                "finish it all.")
                                            msg = RobotSpeech()
                                            msg.status = False
                                            msg.text = text
                                            im.monitor_speech_publisher.publish(msg)

                                        else:
                                            # If new object is from right group
                                            im.instructions_step = 3
                                        break

                            else:
                                # If there is no new object
                                im.instructions_step = 3

                                object_inside = False

                        else:
                            # Checking object added
                            new = im.check_inside()
                            if new and object_inside is False:
                                object_inside = True
                                for g in im.groups:
                                    if new[0] in g[1]:
                                        if next_object < len(im.sequence):
                                            msg = Int64()
                                            msg.data = g[0]
                                            im.added_publisher.publish(msg)

                                            if g[0] == im.sequence[next_object]:
                                                # If new object is from right group
                                                next_object = next_object + 1

                                        break
                            else:
                                if not new and object_inside is True:
                                    # If there is no new object
                                    msg = Int64()
                                    msg.data = 0
                                    im.added_publisher.publish(msg)

                                    object_inside = False

                    if im.instructions_step == 5:
                        if next_object == 3:
                            text = ("The sequence is complete! This is the "
                                    "end of the practice, and you are now "
                                    "ready for the task. You will repeat it a "
                                    "few times and the robot will guide you "
                                    "through it. \n\nMake sure all blocks are "
                                    "in their original places and then click "
                                    "Next to finish the practice.")
                            msg = RobotSpeech()
                            msg.status = False
                            msg.text = text
                            im.monitor_speech_publisher.publish(msg)

                            # Waiting for object to be removed
                            while im.check_inside():
                                im.instructions_step = 5
                                im.show_sequence = False
                                rclpy.spin_once(im)

                            # Updating screen
                            msg = Int64()
                            msg.data = 0
                            im.added_publisher.publish(msg)

                            # Waiting for click in the button
                            while im.instructions_step == 5:
                                im.show_sequence = False
                                rclpy.spin_once(im)

                            # Going to next interaction stage
                            im.interaction_status = 2
                            msg = Int64()
                            msg.data = im.interaction_status
                            im.interaction_status_publisher.publish(msg)

                        else:
                            im.instructions_step = 4
                            last_instruction = 4

            # Robot introduction and instructions
            if im.interaction_status == 2:
                if im.interaction_status != last:
                    last = im.interaction_status

                    # Disabling button to start
                    msg = Bool()
                    msg.data = False
                    im.button_states_publisher.publish(msg)

                    # Configuring condition
                    im.configure_condition(im.conditions_order[condition])

                    if im.logging:
                        if im.conditions_order[condition] == 1:
                            message = "easy-EX"
                        if im.conditions_order[condition] == 2:
                            message = "easy-EXIM"
                        if im.conditions_order[condition] == 3:
                            message = "difficult-EX"
                        if im.conditions_order[condition] == 4:
                            message = "difficult-EXIM"
                        im.write_log(f"CONDITION {condition+1}",
                                     message)
                        im.write_log(f"SEQUENCE {condition+1}",
                                     im.sequence)

                    # Turning robot's eyes on
                    s = socket.socket()
                    s.connect((im.ip_robot_server, im.port_robot_server))
                    command = "Leds on FaceLeds"
                    s.send(bytes(command, 'utf-8'))
                    s.close()

                    time.sleep(1)

                    # Make robot look at the person
                    msg = ObjectIndication()
                    msg.object = 0
                    msg.speed = 0.1
                    im.gaze_command_publisher.publish(msg)

                    time.sleep(1)

                    # Robot introduction
                    text = ("Hi, my name is NAO. Nice to meet you! We are "
                            "going to work together to complete the task "
                            "you just finished practicing. We will do it four "
                            "times, with different sequences each time.")
                    im.speech_routine(text)

                    # Phase instructions
                    text = ("I'll show you the correct sequence on the screen "
                            "once for a few seconds, and you should try to "
                            "memorise it. We'll get 1 point for each correct "
                            "block added, and lose 1 point for each error. "
                            "At any time, you can click the button on the "
                            "screen to see the sequence again, but we lose 2 "
                            "points each time this is done. At the end of the "
                            "task, I will tell you our final score. \n\n"
                            "Click the button on the screen to start with the "
                            "first task.")
                    im.speech_routine(text)

                    # Enabling button to start
                    msg = Bool()
                    msg.data = True
                    im.button_states_publisher.publish(msg)

            # Task execution
            if im.interaction_status == 3:
                if im.interaction_status != last:
                    last = im.interaction_status

                    # Update feedback on screen
                    msg = Int64()
                    msg.data = 0
                    im.added_publisher.publish(msg)

                    # Disabling button to show sequence
                    msg = Bool()
                    msg.data = False
                    im.button_states_publisher.publish(msg)

                    # If necessary, ask to remove blocks
                    if im.check_inside():
                        text = ("Please, put all blocks back to their "
                                "original places.")
                        im.speech_routine(text)

                    # Waiting for object to be removed
                    while im.check_inside():
                        im.show_sequence = False
                        rclpy.spin_once(im)

                    # Reinitialising some variables
                    next_object = 0
                    object_inside = False
                    im.score = 0
                    between_time = 0

                    # If conditions easy-EXIM (2) or difficult-EXIM (4),
                    # make robot look at the person. If not, look forward.
                    if im.conditions_order[condition]%2 == 0:
                        msg = ObjectIndication()
                        msg.object = 0
                        msg.speed = 0.1
                        im.gaze_command_publisher.publish(msg)
                    else:
                        msg = ObjectIndication()
                        msg.object = -1
                        msg.speed = 0.1
                        im.gaze_command_publisher.publish(msg)

                    if condition == 0:
                        # If it's the first condition
                        text = ("Please, look at the screen and I'll show you "
                                "the sequence for a few seconds. Try to "
                                "memorise it and then start adding blocks "
                                "whenever you are ready.")
                    else:
                        # If it's not the first condition
                        text = ("Please, look at the screen to see the "
                                "sequence.")
                    im.speech_routine(text)

                    time.sleep(0.5)

                    if im.logging:
                        im.write_log(f"START {condition}")
                    start_time = time.time()

                    # Show sequence
                    msg = Float64()
                    msg.data = float(im.sequence_time)
                    im.show_sequence_publisher.publish(msg)

                    # Waiting while sequence is shown
                    time.sleep(im.sequence_time)

                    # Enabling button to show sequence
                    msg = Bool()
                    msg.data = True
                    im.button_states_publisher.publish(msg)

                # If there is a command to show the sequence
                if im.show_sequence:
                    if im.logging:
                        im.write_log("SHOW SEQUENCE")

                    im.score = im.score - 2

                    # If conditions easy-EXIM (2) or difficult-EXIM (4),
                    # make robot look at the person.
                    if im.conditions_order[condition]%2 == 0:
                        msg = ObjectIndication()
                        msg.object = 0
                        msg.speed = 0.1
                        im.gaze_command_publisher.publish(msg)

                    text = ("Okay. Look at the screen, I'll show the "
                            "sequence again.")
                    im.speech_routine(text)

                    time.sleep(0.5)

                    msg = Float64()
                    msg.data = float(im.sequence_time)
                    im.show_sequence_publisher.publish(msg)

                    # Waiting while sequence is shown
                    time.sleep(im.sequence_time)

                    im.show_sequence = False
                    between_time = time.time()

                # If conditions easy-EXIM (2) or difficult-EXIM (4)
                # and there is too much time since the last object, look
                # to the person and then to the correct group again.
                if im.conditions_order[condition]%2 == 0:
                    if between_time != 0:
                        if time.time() - between_time > 8:
                            msg = ObjectIndication()
                            msg.object = 0
                            msg.speed = 0.1
                            im.gaze_command_publisher.publish(msg)

                            looking = False

                            time.sleep(3)

                            between_time = 0

                # If conditions easy-EXIM (2) or difficult-EXIM (4),
                # make robot look at next correct group
                if im.conditions_order[condition] % 2 == 0:
                    msg = ObjectIndication()
                    msg.object = im.sequence[next_object]
                    msg.speed = 0.1
                    im.gaze_command_publisher.publish(msg)

                    if not looking:
                        between_time = time.time()
                        if im.logging:
                            im.write_log("GAZE", im.sequence[next_object])

                    looking = True

                # Checking object added
                new = im.check_inside()
                if new and object_inside is False:
                    if im.logging:
                        im.write_log(f"OBJECT ADDED {condition+1}",
                                     new[0])

                    object_inside = True
                    between_time = 0
                    for g in im.groups:
                        if new[0] in g[1]:
                            msg = Int64()
                            msg.data = g[0]
                            im.added_publisher.publish(msg)

                            if g[0] == im.sequence[next_object]:
                                # If new object is from right group
                                next_object = next_object + 1
                                looking = False

                                # Update score
                                im.score = im.score + 1

                                # If conditions easy-EXIM (2) or
                                # difficult-EXIM (4), show happy expression.
                                # If not, just wait the same amount of time.
                                if im.conditions_order[condition]%2 == 0:
                                    msg = FacialExpression()
                                    msg.name = "happy"
                                    msg.duration = 1.0
                                    im.expressions_command_publisher.publish(msg)
                                    time.sleep(1.0)
                                    rclpy.spin_once(im)

                                    # Turning robot's eyes on
                                    s = socket.socket()
                                    s.connect((im.ip_robot_server,
                                               im.port_robot_server))
                                    command = "Leds on FaceLeds"
                                    s.send(bytes(command, 'utf-8'))
                                    s.close()
                                else:
                                    time.sleep(1.0)

                                # If sequence is complete
                                if next_object == im.sequence_length:

                                    # Calculating time to complete task
                                    duration = time.time() - start_time

                                    if im.logging:
                                        im.write_log(f"TIME {condition+1}",
                                                     duration)
                                        im.write_log(f"SCORE {condition+1}",
                                                     im.score)
                                        im.write_log(f"TOTAL ERRORS {condition+1}",
                                                     errors)
                                        im.write_log(f"END {condition+1}",
                                                     "\n")

                                    errors = 0

                                    # If conditions easy-EXIM (2) or
                                    # difficult-EXIM (4), make robot look at
                                    # the person. If not, look forward.
                                    if im.conditions_order[condition]%2 == 0:
                                        msg = ObjectIndication()
                                        msg.object = 0
                                        msg.speed = 0.1
                                        im.gaze_command_publisher.publish(msg)
                                    else:
                                        msg = ObjectIndication()
                                        msg.object = -1
                                        msg.speed = 0.1
                                        im.gaze_command_publisher.publish(msg)

                                    options = ["The sequence is complete. ",
                                               "This is finished. "]
                                    if im.score == 1 or im.score == -1:
                                        text = (random.sample(options, 1)[0] +
                                                (f"The final score on this "
                                                 f"round was {im.score} "
                                                 f"point.\n\n"))
                                    else:
                                        text = (random.sample(options, 1)[0] +
                                                (f"The final score on this "
                                                 f"round was {im.score} "
                                                 f"points.\n\n"))
                                    if condition == 0:
                                        # If it's the first condition
                                        text = (text +
                                                "The screen on your left will "
                                                "soon show a button to open a "
                                                "questionnaire. Please, click "
                                                "the button there and answer "
                                                "the questions with respect "
                                                "to this round of the task. "
                                                "When you're finished, click "
                                                "the button on the main "
                                                "screen to go to the next "
                                                "round.")
                                    elif condition == len(im.conditions_order) - 1:
                                        # If it's the last condition
                                        text = (text +
                                                "Please answer the "
                                                "questionnaire about the task "
                                                "one last time.")
                                    else:
                                        text = (text +
                                                "Again, please answer the "
                                                "questionnaire about this "
                                                "round.")
                                    im.speech_routine(text)

                                    im.interaction_status = im.interaction_status + 1
                                    msg = Int64()
                                    msg.data = im.interaction_status
                                    im.interaction_status_publisher.publish(msg)

                            else:
                                # If new object is not from right group
                                im.score = im.score - 1
                                errors = errors + 1

                                # If conditions easy-EXIM (2) or
                                # difficult-EXIM (4), show sad expression.
                                # If not, just wait the same amount of time.
                                if im.conditions_order[condition]%2 == 0:
                                    msg = FacialExpression()
                                    msg.name = "sad"
                                    msg.duration = 1.5
                                    im.expressions_command_publisher.publish(msg)
                                    time.sleep(1.5)
                                    rclpy.spin_once(im)

                                    # Turning robot's eyes on
                                    s = socket.socket()
                                    s.connect((im.ip_robot_server,
                                               im.port_robot_server))
                                    command = "Leds on FaceLeds"
                                    s.send(bytes(command, 'utf-8'))
                                    s.close()
                                else:
                                    time.sleep(1.5)

                            break
                else:
                    if not new and object_inside is True:
                        # If there is no new object
                        msg = Int64()
                        msg.data = 0
                        im.added_publisher.publish(msg)

                        # Starting timer between objects
                        between_time = time.time()

                        object_inside = False

            # Waiting for questionnaire
            if im.interaction_status == 4:
                if im.interaction_status != last:
                    last = im.interaction_status

                    # Disabling buttons on screen
                    msg = Bool()
                    msg.data = False
                    im.button_states_publisher.publish(msg)

                    # Making the robot look forward
                    msg = ObjectIndication()
                    msg.object = -1
                    msg.speed = 0.1
                    im.gaze_command_publisher.publish(msg)

                    # Showing button to open questionnaire in second screen
                    msg = Int64()
                    msg.data = 1
                    im.screen2_publisher.publish(msg)

                    # Keeping last text on the screen
                    msg = RobotSpeech()
                    msg.status = False
                    msg.text = text
                    im.monitor_speech_publisher.publish(msg)

            # After click of the button indicating questionnaire finished
            if im.interaction_status == 5:
                if im.interaction_status != last:
                    last = im.interaction_status

                    # Updating variables
                    im.score = 0
                    condition = condition + 1

                    if condition < len(im.conditions_order):
                        # If it was not the last round

                        # Configuring new condition
                        im.configure_condition(im.conditions_order[condition])

                        if im.logging:
                            if im.conditions_order[condition] == 1:
                                message = "easy-EX"
                            if im.conditions_order[condition] == 2:
                                message = "easy-EXIM"
                            if im.conditions_order[condition] == 3:
                                message = "difficult-EX"
                            if im.conditions_order[condition] == 4:
                                message = "difficult-EXIM"
                            im.write_log(f"CONDITION {condition + 1}",
                                         message)
                            im.write_log(f"SEQUENCE {condition + 1}",
                                         im.sequence)

                        # Go back to task stage
                        im.interaction_status = 3
                        msg = Int64()
                        msg.data = 3
                        im.interaction_status_publisher.publish(msg)

                        # Showing blank screen in second screen
                        msg = Int64()
                        msg.data = 0
                        im.screen2_publisher.publish(msg)

                    else:
                        # If it was the last round

                        # Make robot look at the person
                        msg = ObjectIndication()
                        msg.object = 0
                        msg.speed = 0.1
                        im.gaze_command_publisher.publish(msg)

                        # Showing blank screen in second screen
                        msg = Int64()
                        msg.data = 0
                        im.screen2_publisher.publish(msg)

                        # Disabling buttons on screen
                        msg = Bool()
                        msg.data = False
                        im.button_states_publisher.publish(msg)

                        text = ("This was the last round. Please, click the "
                                "button on the screen on your left one last "
                                "time to fill a final questionnaire. Then let "
                                "the experimenter know that you're finished. "
                                "\n\nThis was fun! Thank you for "
                                "completing the tasks with me. I will turn "
                                "myself off now. See you next time!")
                        im.speech_routine(text)

                        # Showing button to final questionnaire in 2nd screen
                        msg = Int64()
                        msg.data = 2
                        im.screen2_publisher.publish(msg)

                        # Turning robot's eyes off
                        s = socket.socket()
                        s.connect((im.ip_robot_server, im.port_robot_server))
                        command = "ALLeds off FaceLeds"
                        s.send(bytes(command, 'utf-8'))
                        s.close()

                        # Sending robot to rest pose
                        s = socket.socket()
                        s.connect((im.ip_robot_server, im.port_robot_server))
                        s.send(bytes("Motion rest", 'utf-8'))
                        s.close()

                        # Stop recording
                        if im.record:
                            # Stopping recording using camera from markers
                            record_subprocess.terminate()

                            # Stopping video and audio recording and
                            # starting audio recording through record server
                            s = socket.socket()
                            s.settimeout(4)
                            try:
                                s.connect((im.ip_record, im.port_record))
                                s.send(bytes('stop', 'utf-8'))
                                file = str(s.recv(1024).decode('utf-8'))
                                s.close()
                            except socket.error:
                                pass

                            if im.logging:
                                im.write_log("VIDEO FILE", file)

                            time.sleep(2)

                            s = socket.socket()
                            s.settimeout(4)
                            try:
                                s.connect((im.ip_record, im.port_record))
                                s.send(bytes('start audio', 'utf-8'))
                                s.close()
                            except socket.error:
                                pass

                        # Closing log file
                        if im.logging:
                            im.log_file.close()

            rclpy.spin_once(im)

    except KeyboardInterrupt:
        if im.record:
            record_subprocess.terminate()

            s = socket.socket()
            s.settimeout(4)
            try:
                s.connect((im.ip_record, im.port_record))
                s.send(bytes('stop', 'utf-8'))
                s.close()
            except socket.error:
                pass

        pass
    except Exception as e:
        print(e)


if __name__ == '__main__':
    main()