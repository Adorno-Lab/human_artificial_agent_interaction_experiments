import rospy
import math
import numpy as np
from dqrobotics import *
from nav_msgs.msg import Odometry
from BlockObject import *
from Distances import *


class Interpreter:
    """
    Interpreter is a class to interpret indications of objects or regions of
    interest modeled as BlockObjects. An indicated object is the one that is
    crossed by a certain line and it is closer to an specific point on the line.

    Private attributes:
        _larger: percentage that the objects will be enlarged.
        _poses_indicator: indicates how the objects poses are defined.
        _objects_dimensions: objects fixed dimensions.
        _number_of_objects: number of defined objects.
        _markers_from_reference: objects' markers poses.
        _objects_from_camera: objects' poses with respect to the camera.

    Public attributes:
        x0_camera: transformation from reference frame to the camera.
        objects: list of BlockObjects.

    Private methods:
        _set_fixed_poses: sets the objects' fixed poses with respect to the camera.
        _markers_callback: callback function to get the poses of markers.

    Public methods:
        get_object_pose: returns the updated pose of the object.
        create_block_objects: creates BlockObject objects with updated poses.
        detect_closer_object: finds the object closer to the point on the line.
    """

    def _set_fixed_poses(self, poses_from_camera):
        """
        Sets the fixed poses of the objects with respect to the camera frame.

        :param poses_from_camera: the objects' fixed poses.
        :return:
        """
        for i in range(0, self._number_of_objects):
            p = DQ(poses_from_camera[i][0:3])
            r = DQ(poses_from_camera[i][3:7])
            self._objects_from_camera.append(normalize(r + DQ.E * 0.5 * p * r))

    def _markers_callback(self, msg, index):
        """
        Callback function to get the poses of markers associated with objects.

        :param msg: a nav_msgs/Odometry message with the marker pose.
        :param index: the index of the object associated to the marker.
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
        self._markers_from_reference[index] = normalize(r + DQ.E * 0.5 * p * r)

    def get_object_pose(self, index):
        """
        Returns the updated pose of the object..

        :param index: the index of the object.
        :return: the object's pose.
        """
        if self._poses_indicator == 0:
            lz = self._objects_dimensions[index][2]
            x_marker_center = 1 + 0.5 * DQ.E * DQ([0, 0, -lz / 2]) * 1
            return self._markers_from_reference[index] * x_marker_center
        else:
            return self.x0_camera * self._objects_from_camera[index]

    def create_block_objects(self):
        """
        Creates the BlockObject objects with the updated poses.

        :return:
        """
        self.objects = []
        for i in range(0, self._number_of_objects):
            # Calculating the increase in the dimensions.
            # It will be according to the larger dimension.
            max_dim = self._objects_dimensions[i].index(
                max(self._objects_dimensions[i]))
            plus = self._objects_dimensions[i][max_dim] * self._larger

            # Setting enlarged dimensions.
            lx = self._objects_dimensions[i][0] + plus
            ly = self._objects_dimensions[i][1] + plus
            lz = self._objects_dimensions[i][2] + plus

            # Creating BlockObject object.
            obj = BlockObject(self.get_object_pose(i), lx, ly, lz)
            self.objects.append(obj)

    def detect_closer_object(self, point, line):
        """
        Finds the object closer to the point on the line.

        :param point: point on line.
        :param line: line.
        :return: the index of the closer object or -1 if no object was crossed.
        """
        # Distances on the line from point to the closer plane of each object.
        min_distances_on_line = 999999 * np.ones(self._number_of_objects)

        # Flag indicating if there is at least one object crossed by the line.
        inside_flag = 0

        dt = Distances()

        # For each object, finds the smaller distance related to a point inside
        # the object.
        for i in range(0, self._number_of_objects):
            # First, checks if the line is close to the object.
            if self.objects[i].line_is_close(line) == 1:
                # If close, calculates crossing distances for each of the six
                # planes.
                for j in range(1, 7):
                    if j == 1:
                        crossing_distance = dt.point_to_plane_on_line(
                            point, self.objects[i].pi_top, line)
                    if j == 2:
                        crossing_distance = dt.point_to_plane_on_line(
                            point, self.objects[i].pi_bottom, line)
                    if j == 3:
                        crossing_distance = dt.point_to_plane_on_line(
                            point, self.objects[i].pi_left, line)
                    if j == 4:
                        crossing_distance = dt.point_to_plane_on_line(
                            point, self.objects[i].pi_right, line)
                    if j == 5:
                        crossing_distance = dt.point_to_plane_on_line(
                            point, self.objects[i].pi_front, line)
                    if j == 6:
                        crossing_distance = dt.point_to_plane_on_line(
                            point, self.objects[i].pi_back, line)

                    if crossing_distance != -1:
                        # If the crossing distance is defined and could be
                        # calculated, gets the crossing point between line and
                        # plane.
                        crossing_point = point + crossing_distance * line.P()

                        # Checks if the crossing point is inside the object.
                        if self.objects[i].point_is_inside(crossing_point) == 1:
                            # If inside, holds the least distance and updates
                            # flag.
                            if crossing_distance < min_distances_on_line[i]:
                                min_distances_on_line[i] = crossing_distance
                            inside_flag = 1

        # The closer object will be the one closer to the point according to the
        # distance on the line.
        min_index = np.where(min_distances_on_line == np.amin(min_distances_on_line))
        # Index of the closer object.
        closer_object = min_index[0][0]

        if inside_flag == 0:
            return -1
        else:
            return closer_object

    def __del__(self):
        """
        Destructor of the class.
        Destroys the BlockObject class objects.

        :return:
        """
        del self.objects

    def __init__(self, obj, objects_dimensions, objects_info, larger):
        """
        Constructor of the class.

        :param obj: flag indicating how the objects poses are defined.
        :param objects_dimensions: a list with the objects fixed dimensions.
        :param objects_info: information about the objects poses.
        :param larger: percentage that the objects will be enlarged.
        """

        # Percentage that the objects will be larger than their
        # original dimensions.
        self._larger = larger

        # Indicates how the objects poses are defined.
        # If _poses_indicator == 0: poses with respect to reference frame are
        # obtained from topics.
        # If _poses_indicator == 1: fixed poses with respect to the camera.
        self._poses_indicator = obj

        # Sets the objects fixed dimensions.
        self._objects_dimensions = objects_dimensions

        # Number of objects or regions.
        self._number_of_objects = len(self._objects_dimensions)

        # Transformation between reference frame and the camera.
        self.x0_camera = DQ([1])

        # Initializes list with the BlockObject objects.
        self.objects = []

        # Initializes lists for objects poses.
        self._markers_from_reference = []
        for i in range(0, self._number_of_objects):
            self._markers_from_reference.append(DQ([1]))
        self._objects_from_camera = []

        if self._poses_indicator == 0:
            # Creates the subscribers for the poses with respect to the
            # reference.
            for i in range(0, self._number_of_objects):
                rospy.Subscriber(objects_info[i], Odometry,
                                 self._markers_callback, i)
        if self._poses_indicator == 1:
            # Sets the fixed poses with respect to the camera.
            self._set_fixed_poses(objects_info)
