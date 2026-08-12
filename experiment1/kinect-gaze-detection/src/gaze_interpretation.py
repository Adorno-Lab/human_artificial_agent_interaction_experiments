#!/usr/bin/env python3.6
import roslib
import rospy
from geometry_msgs.msg import Point, PoseStamped
from std_msgs.msg import Int16
from dqrobotics import *
from nav_msgs.msg import Odometry
import math
from Interpreter import *
from BlockObject import *


class Data:
    def points_callback(self, msg, index):
        """
        Callback function to get the facial points.

        :param msg: a geometry_msgs/Point message.
        :param index: the index associated to the point received.
        :return:
        """
        self.facial_points[index] = (msg.x, msg.y)

    def face_callback(self, msg):
        """
        Callback function to get the face detection flag.

        :param msg: a std_msgs/Int16 message.
        :return:
        """
        self.face_flag = msg.data

    def head_callback(self, msg):
        """
        Callback function to get the head pose.

        :param msg: a nav_msgs/Odometry message.
        :return:
        """
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z
        w = msg.pose.pose.orientation.w
        wx = msg.pose.pose.orientation.x
        wy = msg.pose.pose.orientation.y
        wz = msg.pose.pose.orientation.z

        r = DQ([w, wx, wy, wz])
        p = DQ([0, x, y, z])
        self.head = normalize(r + DQ.E * 0.5 * p * r)

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
        # Facial points.
        # facial_points[0]: reference left iris center.
        # facial_points[1]: reference right iris center.
        # facial_points[2]: reference mouth.
        # facial_points[3]: current left iris center.
        # facial_points[4]: current right iris center.
        # facial_points[5]: current mouth.
        self.facial_points = [0] * 6

        # Flag indicating if a face is being detected (1) or not (0).
        self.face_flag = 0

        # Head pose.
        self.head = DQ([1])

        # Pose of the reference frame.
        self.reference_frame = DQ([1])

        # Transformation from camera 1 to human gaze camera.
        p = DQ([1.148716, -0.548599, -3.171204])
        r = DQ([-0.672049, -0.029407, 0.73851, 0.045695])
        #p = DQ([0.967093, -0.536165, -3.054556])
        #r = DQ([0.744782, -0.008013, -0.666242, -0.036846])
        phi_z = - math.pi / 2
        r_z = math.cos(phi_z / 2) + DQ.k * math.sin(phi_z / 2)
        phi_x = - math.pi/2
        r_x = math.cos(phi_x / 2) + DQ.i * math.sin(phi_x / 2)
        r1_camera = r * r_z * r_x
        self.x1_camera = r1_camera + DQ.E * 0.5 * p * r1_camera
        self.x1_camera = normalize(self.x1_camera)

        # Subscribers for the reference values of iris centers and mouth.
        rospy.Subscriber('/ref_left', Point, self.points_callback, 0)
        rospy.Subscriber('/ref_right', Point, self.points_callback, 1)
        rospy.Subscriber('/ref_mouth', Point, self.points_callback, 2)

        # Subscribers for the current values of the iris centers and the mouth.
        rospy.Subscriber('/left_eye', Point, self.points_callback, 3)
        rospy.Subscriber('/right_eye', Point, self.points_callback, 4)
        rospy.Subscriber('/mouth', Point, self.points_callback, 5)

        # Subscriber for the flag indicating if a face is being detected or not.
        rospy.Subscriber('/face', Int16, self.face_callback)

        # Subscriber for the head.
        rospy.Subscriber('/x0_head', Odometry, self.head_callback)

        # Subscriber for the reference frame pose.
        rospy.Subscriber('/frame_0', PoseStamped, self.reference_frame_callback)


def marker_callback(msg, x0_marker):
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    z = msg.pose.pose.position.z
    w = msg.pose.pose.orientation.w
    wx = msg.pose.pose.orientation.x
    wy = msg.pose.pose.orientation.y
    wz = msg.pose.pose.orientation.z

    r = DQ([w, wx, wy, wz])
    p = DQ([0, x, y, z])
    x0_marker[0] = normalize(r + DQ.E * 0.5 * p * r)


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
    rospy.init_node('gaze_interpretation', anonymous=True)

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

    # Head pose with respect to the reference frame.
    x0_head = DQ([1])

    """ If using a marker to know the camera pose:
        
    # Subscriber for the marker of the camera.
    x0_marker = [0]
    rospy.Subscriber('/x09', Odometry, marker_callback, x0_marker)
    # Transformation from marker to camera
    p_marker_camera = DQ([-0.50, 0.37, 0.745])
    phi_x = math.pi / 2
    r_x = math.cos(phi_x / 2) + DQ.i * math.sin(phi_x / 2)
    phi_z = math.pi
    r_z = math.cos(phi_z / 2) + DQ.k * math.sin(phi_z / 2)
    r_marker_camera = r_x * r_z
    x_marker_camera = r_marker_camera + DQ.E * 0.5 * p_marker_camera * r_marker_camera"""

    # Publisher for the looked object.
    object_pub = rospy.Publisher('looked_object', Int16, queue_size=10)

    # Initializing variables.
    ref_left = d.facial_points[0]
    ref_right = d.facial_points[1]
    ref_mouth = d.facial_points[2]

    # Looping at 30 Hz.
    rate = rospy.Rate(30)

    while not rospy.is_shutdown():

        if d.head != 0:
            # Updating camera and head poses.
            x10 = d.reference_frame
            itp.x0_camera = x10.conj() * d.x1_camera
            x0_head = d.head

        """ If using a marker to know the camera pose:
        if x0_marker[0] != 0 and d.head != 0:
            # Updating camera and head poses.
            itp.x0_camera = x0_marker[0] * x_marker_camera
            x0_head = d.head"""

        if d.face_flag == 1 and x0_head != DQ([1]):
            # If head and face are being detected, proceed with interpretation.

            if d.facial_points[0] != ref_left:
                # If the references changed, updates the variables.
                ref_left = d.facial_points[0]
                ref_right = d.facial_points[1]
                ref_mouth = d.facial_points[2]

                # Reference horizontal distance between the iris centers.
                d_eyes_ref = abs(ref_left[0] - ref_right[0])
                if d_eyes_ref == 0:
                    d_eyes_ref = 1

                # Reference vertical distance between eyes and mouth.
                d_em_ref = abs(ref_mouth[1] - (ref_left[1] + ref_right[1]) / 2)
                if d_em_ref == 0:
                    d_em_ref = 1

                # Current pose from camera to the head.
                x_camera_head = itp.x0_camera.conj() * x0_head
                p_camera_head = x_camera_head.translation()
                # Reference distance between camera and head.
                dz_ref = p_camera_head.vec4()[3]

                # Transformation between camera frame and the reference frame of
                # the human.
                r_camera_ref = math.cos(math.pi / 2) + DQ.j * math.sin(math.pi / 2)
                p_camera_ref = x_camera_head.translation()
                x_camera_ref = r_camera_ref + DQ.E * 0.5 * p_camera_ref * r_camera_ref

            if d.facial_points[3]:
                # If detection of the facial points already started.

                left_center = d.facial_points[3]
                right_center = d.facial_points[4]
                mouth = d.facial_points[5]

                # Current pose from camera to the head.
                x_camera_head = itp.x0_camera.conj() * x0_head
                p_camera_head = x_camera_head.translation()
                # Current distance between camera and head.
                dz_ref_now = p_camera_head.vec4()[3]

                # Updating the reference facial distances according to the
                # change in the distance between camera and head.
                d_eyes_ref_now = d_eyes_ref * dz_ref / dz_ref_now
                d_em_ref_now = d_em_ref * dz_ref / dz_ref_now

                # Current horizontal distance between the iris centers.
                d_eyes = abs(left_center[0] - right_center[0])

                # Rotation angle theta (left and right).
                cos_value = d_eyes / d_eyes_ref_now
                if cos_value > 1:
                    cos_value = 1
                if cos_value < -1:
                    cos_value = -1
                theta = math.acos(cos_value)
                # Rotation around y-axis.
                if mouth[0] > ref_mouth[0]:
                    # If mouth went right on image, then real mouth went left.
                    ry = math.cos(-theta / 2) + DQ.j * math.sin(-theta / 2)
                elif mouth[0] < ref_mouth[0]:
                    # If mouth went left on image, then real mouth went right.
                    ry = math.cos(theta / 2) + DQ.j * math.sin(theta / 2)
                else:
                    ry = DQ([1])

                # Current vertical distance between eyes and mouth.
                d_em = abs(mouth[1] - (left_center[1] + right_center[1]) / 2)

                # Rotation angle phi (up and down).
                cos_value = d_em / d_em_ref_now
                if cos_value > 1:
                    cos_value = 1
                if cos_value < -1:
                    cos_value = -1
                phi = math.acos(cos_value)
                # Rotation around x-axis.
                if mouth[1] > ref_mouth[1]:
                    # If mouth went down on image, then angle < 0.
                    rx = math.cos(-phi / 2) + DQ.i * math.sin(-phi / 2)
                elif mouth[1] < ref_mouth[1]:
                    # If mouth went up on image, then angle > 0.
                    rx = math.cos(phi / 2) + DQ.i * math.sin(phi / 2)
                else:
                    rx = DQ([1])

                # Human gaze direction after the head rotation with respect to
                # the reference human frame.
                direction_ref = Ad(ry * rx, DQ.k)

                # Human gaze direction with respect to the reference frame.
                x0_ref = itp.x0_camera * x_camera_ref
                r0_ref = x0_ref.rotation()
                direction_0 = Ad(r0_ref, direction_ref)

                # Using the point of the head as the point of the eyes.
                x0_eyes = x0_head
                p0_eyes = x0_eyes.translation()

                # Creating the gaze line with respect to reference frame.
                gaze_line = direction_0 + DQ.E * (cross(p0_eyes, direction_0))

                # Defines BlockObject objects using updated poses.
                itp.create_block_objects()

                # Detects the looked object (or region).
                closer = itp.detect_closer_object(p0_eyes, gaze_line)

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
            # If no face is being detected, resets the references.
            ref_left = 0
            ref_right = 0
            ref_mouth = 0

        rate.sleep()

    # Destroying all the classes objects.
    del d
    del itp


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass