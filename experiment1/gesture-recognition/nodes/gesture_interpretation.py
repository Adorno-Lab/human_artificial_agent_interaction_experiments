#!/usr/bin/env python3.6
import roslib
import rospy
from math import pow, sqrt
import time
from dqrobotics import *
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from std_msgs.msg import Int16
from BlockObject import *
from Interpreter import *
from Distances import *
from JointsTracker import *


class Data:
    def detecting_callback(self, msg):
        """
        Callback function to get the human detection flag.

        :param msg: a std_msgs/Int16 message.
        :return:
        """
        self.detecting_flag = msg.data

    def reference_frame_callback(self, msg):
        """
        Callback function to get the pose of the reference frame with respect
        to camera 1.

        :param msg: a geometry_msgs/PoseStamped message.
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
        self.reference_frame = normalize(r + DQ.E * 0.5 * p * r)

    def __init__(self):
        # Flag indicating if a human is being tracked (1) or not (0).
        self.detecting_flag = 0

        # Pose of the reference frame.
        self.reference_frame = DQ([1])

        # Transformation from camera 1 to camera 2.
        r12 = DQ([0.99614, -0.00160378, 0.0227897, 0.0158056])
        p12 = DQ([0, -0.0189038, 0.0227446, -1.76328])
        self.x12 = r12 + DQ.E * 0.5 * p12 * r12
        self.x12 = normalize(self.x12)

        # Transformation from camera 2 to tracker camera.
        #r2camera = DQ([-0.766528, -0.017566, 0.638758, 0.064154])
        #p2camera = DQ([0, 1.069735, 0.010812, -1.690578])
        #self.x2_camera = r2camera + DQ.E * 0.5 * p2camera * r2camera
        #self.x2_camera = DQ([0.774683, -0.014779, -0.622946, -0.107637,
        #                    -0.027973, -0.166321, 0.134801, -0.958648])
        #self.x2_camera = normalize(self.x2_camera)

        # Transformation from camera 1 to tracker camera.
        #self.x1_camera = self.x12 * self.x2_camera
        self.x1_camera = normalize(DQ([-0.74823, -0.025314, 0.662098, 0.033731,
                                       0.086797, 0.813125, 0.038706, 1.775807]))

        # Subscriber for the flag indicating if a human is being tracked or not.
        rospy.Subscriber('/detecting_flag', Int16, self.detecting_callback)

        # Subscriber for the reference frame pose.
        rospy.Subscriber('/frame_0', PoseStamped, self.reference_frame_callback)


def gesture_recognition(x_lefthip, x_lefthand, x_righthip, x_righthand):
    """
    Calculates hip-hand distances and compare them with the predefined limits
    to determine if the human is pointing or not.
        g = 0: standing
        g = 1: left pointing*
        g = 2: right pointing*

    * 'Left' and 'right' are relative to the camera.

    :param x_lefthip: left hip pose.
    :param x_lefthand: left hand pose.
    :param x_righthip: right hip pose.
    :param x_righthand: right hand pose.
    :return: index of the gesture.
    """

    # Reference hip-hand distances for each gesture.
    # Standing:
    GESTURE_1_LEFT = 0.250624936155728
    GESTURE_1_RIGHT = 0.2193689970138194
    # Left pointing:
    GESTURE_2_LEFT = 0.8417310626216875
    GESTURE_2_RIGHT = 0.20895394289498367
    # Right pointing:
    GESTURE_3_LEFT = 0.1966524598046372
    GESTURE_3_RIGHT = 0.8520630414285886

    gestures = {
        0: "Standing",
        1: "Image-left pointing",
        2: "Image-right pointing"
    }

    # Getting the joints positions.
    lefthip = x_lefthip.translation()
    lefthand = x_lefthand.translation()
    righthip = x_righthip.translation()
    righthand = x_righthand.translation()

    # Calculating the current distances.
    hhl = norm(lefthip - lefthand)
    hhr = norm(righthip - righthand)
    hhl = hhl.vec4()[0]
    hhr = hhr.vec4()[0]

    if hhl < 1.5*GESTURE_1_LEFT and hhr < 1.5*GESTURE_1_RIGHT:
        # If both distances (left and right) are smaller than the limits.
        g = 0  # Standing.
    else:
        if hhl > hhr:
            # If the left distance is greater than the right distance.
            g = 1  # Left pointing.
        else:
            g = 2  # Right pointing.

    return g


def string_to_list(s, element_type='STR'):
    """
    Converts a string into a two-dimensional list.

    The rows of the list are separated by semicolons and the columns by colons.

    :param s: the string to be converted
    :param element_type: the type of the elements in the final list.
    :return:
    """
    # Splits the rows, creating a one-dimensional list of strings.
    rows = list(s.split(";"))

    final_list = []
    for i in range(0, len(rows)):
        # Splits each row in its columns.
        cols = list(rows[i].split(","))
        if len(cols) == 1:
            cols = cols[0]

        # Creating the final list with the correct types of elements.
        if element_type == 'STR':
            final_list.append(cols)
        if element_type == 'INT':
            final_list.append(list(map(int, cols)))
        if element_type == 'FLOAT':
            final_list.append(list(map(float, cols)))

    return final_list


def main():
    rospy.init_node('gesture_interpretation', anonymous=True)

    # Percentage that the objects will be larger than their original dimensions.
    larger = float(rospy.get_param('~larger'))

    # Objects dimensions.
    objects_dimensions_str = rospy.get_param("~objects_dimensions")
    objects_dimensions = string_to_list(objects_dimensions_str, 'FLOAT')

    # Indicates how the objects poses will be defined.
    # If (0), objects poses with respect to the reference frame are obtained
    # through topics.
    # If (1), objects poses are fixed and given with respect to the camera
    # frame.
    obj = int(rospy.get_param('~objects'))

    # Creating the Interpreter object.
    if obj == 0:
        topic_names_str = rospy.get_param('~topics')
        topic_names = string_to_list(topic_names_str)

        itp = Interpreter(obj, objects_dimensions, topic_names, larger)
    if obj == 1:
        fixed_poses_str = rospy.get_param('~fixed_poses')
        fixed_poses = string_to_list(fixed_poses_str, 'FLOAT')

        itp = Interpreter(obj, objects_dimensions, fixed_poses, larger)

    # Number of object detections that should occur before calling it an
    # indication.
    detection_number = int(rospy.get_param('~detection_number'))

    # List with the last objects detected.
    last_detected = [-1] * detection_number

    d = Data()

    tracker = JointsTracker()

    # Flag indicating if the human is pointing (1 or 2) or not (0).
    pointing = 0

    # Publishers for pointing flag and pointed object.
    pointing_pub = rospy.Publisher('pointing_flag', Int16, queue_size=10)
    object_pub = rospy.Publisher('pointed_object', Int16, queue_size=10)

    # Looping at 50 Hz.
    rate = rospy.Rate(50)

    while not rospy.is_shutdown():
        if d.detecting_flag == 1:
            # If a human is being tracked.

            # Updating human joints' poses with respect to the reference frame.
            tracker.update()

            # Obtaining camera pose with respect to the reference frame.
            x10 = d.reference_frame
            itp.x0_camera = x10.conj() * d.x1_camera
            tracker.x0_camera = itp.x0_camera

            # Poses of human joints of interest with respect to reference frame.
            x_0_lefthip = tracker.joints_from_reference[9]
            x_0_leftelbow = tracker.joints_from_reference[4]
            x_0_lefthand = tracker.joints_from_reference[5]
            x_0_righthip = tracker.joints_from_reference[12]
            x_0_rightelbow = tracker.joints_from_reference[7]
            x_0_righthand = tracker.joints_from_reference[8]

            # Gesture recognition:
            # If g == 0: human standing.
            # If g == 1: human pointing with left arm of the image.
            # If g == 2: human pointing with right arm of the image.
            g = gesture_recognition(x_0_lefthip, x_0_lefthand,
                                    x_0_righthip, x_0_righthand)
            if g != 0:
                # If the human is pointing.
                if g == 1:
                    # Line direction: from elbow to hand.
                    direction = x_0_lefthand.translation() - x_0_leftelbow.translation()
                    direction = normalize(direction)
                    # Point on the line.
                    point = x_0_leftelbow.translation()
                    pointing = 1
                if g == 2:
                    # Line direction: from elbow to hand.
                    direction = x_0_righthand.translation() - x_0_rightelbow.translation()
                    direction = normalize(direction)
                    # Point on the line.
                    point = x_0_rightelbow.translation()
                    pointing = 2

                # Creating pointing line with respect to the reference frame.
                pointing_line = direction + DQ.E*(cross(point, direction))

                # Defines BlockObject objects using updated poses.
                itp.create_block_objects()

                # Detect the indicated object (or region).
                closer = itp.detect_closer_object(point, pointing_line)

                if closer != -1:
                    # If any object was indicated.
                    if last_detected.count(closer + 1) == len(last_detected):
                        # If the object is recognized a few times in a row,
                        # then its number is published in the topic.
                        object_pub.publish(closer + 1)

                # Updating the stored list.
                for i in range(0, len(last_detected) - 1):
                    last_detected[i] = last_detected[i + 1]
                last_detected[len(last_detected) - 1] = closer + 1
            else:
                # If the human is not pointing.
                pointing = 0

            # Publishes the flag indicating if human is pointing or not.
            pointing_pub.publish(pointing)

        rate.sleep()

    # Destroying all the classes objects.
    del d
    del tracker
    del itp


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
