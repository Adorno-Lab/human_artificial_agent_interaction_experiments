import roslib
import rospy
from dqrobotics import *
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry


class JointsTracker:
    """
    JointsTracker is a class to deal with the tracking of the human joints.

    It reads the joints poses with respect to the camera, transforms them to the
    point of view of the reference frame, and publishes the poses.

    Public attributes:
        joints_from_camera: pose of the joints with respect to the camera.
        joints_from_reference: pose of the joints with respect to the reference.
        x0_camera: transformation from reference frame to camera frame.
        publishers: list of publishers for the joint poses with respect to the
                    reference frame.

    Public methods:
        joints_callback: callback function to get the poses of human joints.
        update: calculates poses with respect to the reference frame and
                publishes them.
    """
    def joints_callback(self, msg, index):
        """
        Callback function to get the poses of human joints.

        :param msg: a geometry_msgs/PoseStamped message.
        :param index: the index associated to the point received.
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
        self.joints_from_camera[index] = normalize(r + DQ.E * 0.5 * p * r)

    def update(self):
        """
        Calculates the human joints poses with respect to the reference frame
        and publishes the information.

        :return:
        """
        x0_camera = self.x0_camera

        if x0_camera != DQ([1]):
            x_camera_joint = self.joints_from_camera.copy()

            for i in range(0, 15):
                # Calculating joint pose from reference.
                x0_joint = x0_camera * x_camera_joint[i]

                # Obtaining position and rotation.
                p = x0_joint.translation().vec4()
                r = x0_joint.P().vec4()

                # Setting Odometry message to publish it.
                message = Odometry()
                message.header.frame_id = "/Frame_marker_0"
                message.pose.pose.position.x = p[1]
                message.pose.pose.position.y = p[2]
                message.pose.pose.position.z = p[3]
                message.pose.pose.orientation.w = r[0]
                message.pose.pose.orientation.x = r[1]
                message.pose.pose.orientation.y = r[2]
                message.pose.pose.orientation.z = r[3]
                self.publishers[i].publish(message)

                self.joints_from_reference[i] = x0_joint

    def __init__(self):
        """
        Order of the joints in all the list attributes:
        ('left' and 'right' are relative to the camera.)

            [0]: head pose.
            [1]: neck pose.
            [2]: torso pose.
            [3]: left shoulder pose.
            [4]: left elbow pose.
            [5]: left hand pose.
            [6]: right shoulder pose.
            [7]: right elbow pose.
            [8]: right hand pose.
            [9]: left hip pose.
            [10]: left knee pose.
            [11]: left foot pose.
            [12]: right hip pose.
            [13]: right knee pose.
            [14]: right foot pose.
        """

        # Pose of the joints with respect to the camera.
        self.joints_from_camera = [DQ([1])] * 15

        # Pose of the joints with respect to the reference.
        self.joints_from_reference = [DQ([1])] * 15

        # Transformation from reference frame to camera frame.
        self.x0_camera = DQ([1])

        # Publishers for the joint poses with respect to the reference frame.
        self.publishers = [rospy.Publisher('x0_head', Odometry, queue_size=1),
                           rospy.Publisher('x0_neck', Odometry, queue_size=1),
                           rospy.Publisher('x0_torso', Odometry, queue_size=1),
                           rospy.Publisher('x0_left_shoulder', Odometry, queue_size=1),
                           rospy.Publisher('x0_left_elbow', Odometry, queue_size=1),
                           rospy.Publisher('x0_left_hand', Odometry, queue_size=1),
                           rospy.Publisher('x0_right_shoulder', Odometry, queue_size=1),
                           rospy.Publisher('x0_right_elbow', Odometry, queue_size=1),
                           rospy.Publisher('x0_right_hand', Odometry, queue_size=1),
                           rospy.Publisher('x0_left_hip', Odometry, queue_size=1),
                           rospy.Publisher('x0_left_knee', Odometry, queue_size=1),
                           rospy.Publisher('x0_left_foot', Odometry, queue_size=1),
                           rospy.Publisher('x0_right_hip', Odometry, queue_size=1),
                           rospy.Publisher('x0_right_knee', Odometry, queue_size=1),
                           rospy.Publisher('x0_right_foot', Odometry, queue_size=1)]

        # Subscribers for the human joints.
        rospy.Subscriber('/head', PoseStamped, self.joints_callback, 0)
        rospy.Subscriber('/neck', PoseStamped, self.joints_callback, 1)
        rospy.Subscriber('/torso', PoseStamped, self.joints_callback, 2)
        rospy.Subscriber('/left_shoulder', PoseStamped, self.joints_callback, 3)
        rospy.Subscriber('/left_elbow', PoseStamped, self.joints_callback, 4)
        rospy.Subscriber('/left_hand', PoseStamped, self.joints_callback, 5)
        rospy.Subscriber('/right_shoulder', PoseStamped, self.joints_callback, 6)
        rospy.Subscriber('/right_elbow', PoseStamped, self.joints_callback, 7)
        rospy.Subscriber('/right_hand', PoseStamped, self.joints_callback, 8)
        rospy.Subscriber('/left_hip', PoseStamped, self.joints_callback, 9)
        rospy.Subscriber('/left_knee', PoseStamped, self.joints_callback, 10)
        rospy.Subscriber('/left_foot', PoseStamped, self.joints_callback, 11)
        rospy.Subscriber('/right_hip', PoseStamped, self.joints_callback, 12)
        rospy.Subscriber('/right_knee', PoseStamped, self.joints_callback, 13)
        rospy.Subscriber('/right_foot', PoseStamped, self.joints_callback, 14)
