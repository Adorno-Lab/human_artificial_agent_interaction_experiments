from dqrobotics import *
from Distances import *


class BlockObject:
    """
    CLASS BlockObject

    This class is used to define a model for an object in space using dual
    quaternion planes. Six planes are used and their normal point to the
    interior of the object.

    Way of defining an object:
        Parameters for the object are the pose of its center and its lengths
        parallel to the objects' axes. Set the new object using:
            >>> obj = BlockObject(x0_object,lx,ly,lz),
        where x0_object is the transformation from the reference frame to the
        frame in the center of the object and lx, ly and lz are scalars for the
        lengths.

    Public attributes:
        x0_object: pose of the center of the object in the point of view of reference frame.
        length_x: scalar length in object's x direction.
        length_y: scalar length in object's y direction.
        length_z: scalar length in object's z direction.
        pi_top: plane for the top of the object, related to z-axis.
        pi_bottom: plane for the bottom of the object, related to z-axis.
        pi_left: plane for the left of the object, related to y-axis.
        pi_right: plane for the right of the object, related to y-axis.
        pi_front: plane for the front of the object, related to x-axis.
        pi_back: plane for the back of the object, related to x-axis.

    Private methods:
        _define_planes: defines the six planes of the object.
    Public methods:
        line_is_close: checks if a line is close to the object (1) or not (0).
        point_is_inside: checks if a point is inside the object (1) or not (0).

    """

    def _define_planes(self):
        """
        Defines the planes that describe the BlockObject.

        The planes are first defined in the point of view of the object's frame
        and then are transformed to the point of view of reference frame.

        :return:
        """

        # Top plane (parallel to xy-plane).
        normal = DQ([0, 0, -1])
        point = DQ([0, 0, 0, self.length_z / 2])
        plane = normal + DQ.E * dot(normal, point)
        self.pi_top = Adsharp(self.x0_object, plane)

        # Bottom plane (parallel to xy-plane).
        normal = DQ([0, 0, 1])
        point = DQ([0, 0, 0, -self.length_z / 2])
        plane = normal + DQ.E * dot(normal, point)
        self.pi_bottom = Adsharp(self.x0_object, plane)

        # Left plane (parallel to xz-plane).
        normal = DQ([0, 1, 0])
        point = DQ([0, 0, -self.length_y / 2, 0])
        plane = normal + DQ.E * dot(normal, point)
        self.pi_left = Adsharp(self.x0_object, plane)

        # Right plane (parallel to xz-plane).
        normal = DQ([0, -1, 0])
        point = DQ([0, 0, self.length_y / 2, 0])
        plane = normal + DQ.E * dot(normal, point)
        self.pi_right = Adsharp(self.x0_object, plane)

        # Front plane (parallel to yz-plane).
        normal = DQ([-1, 0, 0])
        point = DQ([0, self.length_x / 2, 0, 0])
        plane = normal + DQ.E * dot(normal, point)
        self.pi_front = Adsharp(self.x0_object, plane)

        # Back plane (parallel to yz-plane).
        normal = DQ([1, 0, 0])
        point = DQ([0, -self.length_x / 2, 0, 0])
        plane = normal + DQ.E * dot(normal, point)
        self.pi_back = Adsharp(self.x0_object, plane)

    def line_is_close(self, line):
        """
        Checks if a line is close to the object.

        Being close means inside a sphere centered in the object's center and
        with the largest length among x, y and z lengths as radius.

        :param line: the DQ Plucker line to be verified.
        :return: 1 if close, 0 if not.
        """
        # Sphere's radius.
        radius = max([self.length_x, self.length_y, self.length_z])

        # Position of the object's center.
        center = self.x0_object.translation()

        dt = Distances()
        if dt.line_to_point_distance(line, center) <= radius:
            return 1
        else:
            return 0

    def point_is_inside(self, point):
        """
        Checks if a point is inside the object.

        The point is inside the object if it is on or in the positive side of
        all of the object's six planes.

        :param point: the quaternion point to be verified.
        :return: 1 if inside, 0 if not.
        """
        dt = Distances()
        if dt.plane_to_point_distance(self.pi_top, point) >= 0:
            if dt.plane_to_point_distance(self.pi_bottom, point) >= 0:
                if dt.plane_to_point_distance(self.pi_left, point) >= 0:
                    if dt.plane_to_point_distance(self.pi_right, point) >= 0:
                        if dt.plane_to_point_distance(self.pi_front, point) >= 0:
                            if dt.plane_to_point_distance(self.pi_back, point) >= 0:
                                return 1
        return 0

    def __init__(self, x0_object, lx, ly, lz):
        """
        BlockObject constructor.

        :param x0_object: pose of the center of the object in the point of view
                          of the reference frame.
        :param lx: scalar length in object's x direction.
        :param ly: scalar length in object's y direction.
        :param lz: scalar length in object's z direction.
        :return:
        """
        self.x0_object = x0_object
        self.length_x = lx
        self.length_y = ly
        self.length_z = lz

        self._define_planes()