from dqrobotics import *


class Distances:
    """
    Distances is a class with methods to calculate some distances between
    geometric primitives using dual quaternion algebra.

    Public methods:
        line_to_point_distance: calculates absolute distance between line and
                                point.
        plane_to_point_distance: calculates signed distance between plane and
                                 point from the point of view of the plane.
        point_to_plane_on_line: calculates absolute distance between point and
                                plane on line direction.
    """

    @staticmethod
    def line_to_point_distance(line, point):
        """
        Calculates absolute distance between line and point.

        The distance is calculated using Equation 21 of Marinho et al., 2018.*

        * Marinho, M. M., Adorno, B. V., Harada, K., & Mitsuishi, M. (2018).
        Active Constraints Using Vector Field Inequalities for Surgical Robots.
        2018 IEEE International Conference on Robotics and Automation (ICRA),
        5364-5371. https://doi.org/10.1109/ICRA.2018.8461105

        :param line: unit pure dual quaternion Plucker line.
        :param point: pure quaternion point.
        :return: calculated distance.
        """
        if is_line(line) and is_pure_quaternion(point):
            # If the parameters are correct, calculates distance.
            distance = norm(cross(point, line.P()) - line.D())
            vec_distance = distance.vec8()
            return vec_distance[0]
        else:
            point_vec = point.vec4()
            if 1e-13 < point_vec[0] < 1e-11:
                # If the real part is very small, ignores it and calculates
                # distance.
                point_vec[0] = 0
                point = DQ(point_vec)
                if is_line(line) and is_pure_quaternion(point):
                    distance = norm(cross(point, line.P()) - line.D())
                    vec_distance = distance.vec8()
                    return vec_distance[0]
            else:
                # If the real part can not be ignored, returns error message.
                print('>>>>>> ERROR line_to_point_distance(): ' +
                      'The function line_to_point_distance() accepts only' +
                      ' line (unit dual pure quaternion) and point (pure' +
                      ' quaternion) elements.')
                print("line: " + str(line))
                print("point: " + str(point))
                return -1

    @staticmethod
    def plane_to_point_distance(plane, point):
        """
        Calculates signed distance between plane and point from the point of
        view of the plane.

        The distance is calculated using Equation 15 of Marinho et al., 2018.*

        * Marinho, M. M., Adorno, B. V., Harada, K., & Mitsuishi, M. (2018).
        Active Constraints Using Vector Field Inequalities for Surgical Robots.
        2018 IEEE International Conference on Robotics and Automation (ICRA),
        5364-5371. https://doi.org/10.1109/ICRA.2018.8461105

        :param plane: dual quaternion plane.
        :param point: pure quaternion point.
        :return: calculated distance.
        """
        if is_plane(plane) and is_pure_quaternion(point):
            # If the parameters are correct, calculates distance.
            distance = dot(point, plane.P()) - plane.D()
            vec_distance = distance.vec8()
            return vec_distance[0]
        else:
            point_vec = point.vec4()
            planedual_vec = plane.D().vec4()
            if 1e-13 < point_vec[0] < 1e-11 or planedual_vec[3] < 1e-11:
                # If there are small residues, ignores them and calculates
                # distance. The plane dual part should have only real part but
                # sometimes it has a residue in planedual_vec[3].
                point_vec[0] = 0
                point = DQ(point_vec)
                planedual_vec[3] = 0
                plane = plane.P() + DQ.E * DQ(planedual_vec)
                if is_plane(plane) and is_pure_quaternion(point):
                    distance = dot(point, plane.P()) - plane.D()
                    vec_distance = distance.vec8()
                    return vec_distance[0]
            else:
                # If the residues can not be ignored, returns error message.
                print('>>>>>> ERROR plane_to_point_distance(): ' +
                      'The function plane_to_point_distance() accepts only' +
                      ' plane (unit dual quaternion with real dual part) ' +
                      'and point (pure quaternion) elements.')
                print("plane: " + str(plane))
                print("point: " + str(point))
                return -1

    @staticmethod
    def point_to_plane_on_line(point, plane, line):
        """
        Calculates absolute distance between point and plane on line direction.

        The distance is calculated using Equation 2 of Campos & Adorno, 2020.*

        * Campos, A. C. A. & Adorno, B. V. (2020). Development of Human-Robot
        Communication Technologies for Future Interaction Experiments. 2020
        17th IEEE Latin American Robotics Symposium / 8th Brazilian Symposium
        of Robotics (LARS/SBR 2020).

        :param point: pure quaternion point.
        :param plane: dual quaternion plane.
        :param line: unit pure dual quaternion Plucker line.
        :return: calculated distance.
        """
        if is_pure_quaternion(point) and is_plane(plane) and is_line(line):
            # If the parameters are correct, calculates distance.
            if dot(line.P(), plane.P()) != 0:
                # If line and plane are not parallel, calculates distance.
                num = plane.D() - dot(point, plane.P())
                num_vec = num.vec8()
                den = dot(line.P(), plane.P())
                den_vec = den.vec8()
                d = num_vec[0] / den_vec[0]
                distance = abs(d)
                return distance
            else:
                # If line and plane are parallel, distance is not defined.
                return -1
        else:
            point_vec = point.vec4()
            planedual_vec = plane.D().vec4()
            if 1e-13 < point_vec[0] < 1e-11 or planedual_vec[3] < 1e-11:
                # If there are small residues, ignores them and calculates
                # distance. The plane dual part should have only real part but
                # sometimes it has a residue in planedual_vec[3].
                point_vec[0] = 0
                point = DQ(point_vec)
                planedual_vec[3] = 0
                plane = plane.P() + DQ.E * DQ(planedual_vec)
                if dot(line.P(), plane.P()) != 0:
                    # If line and plane are not parallel, calculates distance.
                    num = plane.D() - dot(point, plane.P())
                    num_vec = num.vec8()
                    den = dot(line.P(), plane.P())
                    den_vec = den.vec8()
                    d = num_vec[0] / den_vec[0]
                    distance = abs(d)
                    return distance
                else:
                    # If line and plane are parallel, distance is not defined.
                    return -1
            else:
                # If the residues can not be ignored, returns error message.
                print('>>>>>> ERROR point_to_plane_on_line(): ' +
                      'The function point_to_plane_on_line() accepts only' +
                      ' point (pure quaternion), plane (unit dual' +
                      ' quaternion with real dual part) and line (unit pure' +
                      ' dual quaternion) elements.')
                print("point: " + str(point))
                print("plane: " + str(plane))
                print("line: " + str(line))
                return -1

