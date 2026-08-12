from tkinter import *
from PIL import Image, ImageTk
import cv2
import time
import os.path
from os import path
from dqrobotics import *
import math
import rospy
import rospkg
from std_msgs.msg import String, Int16
from nav_msgs.msg import Odometry
import numpy as np
from random import sample
from overlay_transparent import *
from playsound import playsound
from threading import Thread


class VirtualAgent:
    """
    VirtualAgent is a class for the virtual agents' application.

    Private attributes:
        _pkg_path: path for the ROS package.
        _black: color for the background.
        _window: application window.
        _screen_px: screen dimensions in pixels.
        _screen_m: screen dimensions in meters.
        _window_px: window dimensions in pixels.
        _window_position: window position in pixels.
        _original_eyes_position_y: original y coordinate of the eyes image.
        _eyes_position_y: y coordinate of the eyes image after resize.
        _original_width and _original_height: original image dimensions.
        _image_width and _image_height: final image dimensions.
        _luna_images: images of the virtual agents' faces.
        _luna_eyes: images of the virtual agents' eyes looking to the corners.
        _original_eyecenter: eyes center point in the original images.
        _eyes_px: eyes center point in the final images.
        _images and _eyes_images: current sets of images.
        _image: current image being shown.
        _photo: current ImageTk.PhotoImage image.
        _current_va: current virtual agent.
        _canvas: canvas object of the window.
        _time_open and _time_closed: times for the blinking animation.
        _time_talking: time for the mouth animation.
        _eyes_state: current state of the virtual agent's eyes.
        _corner_eyes: indicates if the eyes should be changed.
        _mouth_index: index of the current mouth shape while talking.
        _now_time, _eyes_start_time, and _mouth_start_time: times for animations.
        _audio: audio data.
        _expression: index of the current expression.
        _ref_point: reference point in front of the virtual agent.
        _x_eyes_ref: reference point with respect to the eyes point.
        _x0_goal: goal point with respect to marker 0.
        _goal_point and _x_eyes_goal: goal point with respect to the eyes.
        _tl, _bl, _tr, and _br: image corner points.
        _ref_corners: reference corner points.
        _start_corners: corner points at the beginning of the transformation.
        _final_corners: final corner points.
        _depth_points: points to be used to create the depth.
        _iterations: iteration data for transformation.
        _x_marker_eyes: transformation from marker 0 to the eyes.
        _x0_marker: transformation from marker 0 to virtual agent's marker.
        _x_tl, _x_bl, _x_tr, and _x_br: corner points with respect to the eyes.

    Public attributes:
        Luna: virtual agents.

    Private methods:
        _configure_images: loads and resizes the virtual agents images.
        _define_eyes_point: defines the eyes point related to screen's TL point.
        _convert_image: converts a Mat image in a ImageTk.PhotoImage image.
        _face_and_voice: deals with the virtual agent's expressions and voice.
        _point_to_plane_on_line: calculates distance between point and plane.
        _define_depth_points: defines the points to create depth.
        _update_canvas: updates the Canvas in the window.
        _define_transformation: define the image transformation.
        _gaze: deals with the virtual agent's gaze.
        _expression_callback: callback method for the expressions subscriber.
        _voice_callback: callback method for the voice subscriber.
        _gaze_callback: callback method for the gaze subscriber.
        _marker_callback: callback method for the marker subscriber.

    Public methods:
        update_all: updates the application.
        destroy_all: destroys the main window.
        set_virtual_agent: updates the virtual agent being used.

    Published topics:
        /va_speaking: to publish if the virtual agent is speaking or not.
    Subscribed topics:
        /virtual_agent: to get command to change the virtual agent.
        /expression: to get expression commands.
        /voice: to get voice commands.
        /va_gaze: to get gaze commands.
    """

    def _configure_images(self, name):
        """
        Loads and resizes the images of the virtual agent.

        :param name: the name of the virtual agent.
        :return: the adjusted images for the face and eyes.
        """
        path = self._pkg_path + '/src/images/' + name + '/'
        neutral = [cv2.imread(path + 'neutral.png'),
                   cv2.imread(path + 'neutral_blink.png'),
                   cv2.imread(path + 'neutral_talk1.png'),
                   cv2.imread(path + 'neutral_talk2.png'),
                   cv2.imread(path + 'neutral_talk3.png')]
        eyebrows = [cv2.imread(path + 'eyebrows.png'),
                    cv2.imread(path + 'eyebrows_blink.png'),
                    cv2.imread(path + 'eyebrows_talk1.png'),
                    cv2.imread(path + 'eyebrows_talk2.png'),
                    cv2.imread(path + 'eyebrows_talk3.png')]
        happy = [cv2.imread(path + 'happy.png'),
                 cv2.imread(path + 'happy_blink.png'),
                 cv2.imread(path + 'happy_talk1.png'),
                 cv2.imread(path + 'happy_talk2.png'),
                 cv2.imread(path + 'happy_talk3.png')]
        confused = [cv2.imread(path + 'confused.png'),
                    cv2.imread(path + 'confused_blink.png'),
                    cv2.imread(path + 'confused_talk1.png'),
                    cv2.imread(path + 'confused_talk2.png'),
                    cv2.imread(path + 'confused_talk3.png')]
        thinking = [cv2.imread(path + 'thinking.png'),
                    cv2.imread(path + 'thinking_blink.png'),
                    cv2.imread(path + 'thinking_talk1.png'),
                    cv2.imread(path + 'thinking_talk2.png'),
                    cv2.imread(path + 'thinking_talk3.png')]
        sad = [cv2.imread(path + 'sad.png'),
               cv2.imread(path + 'sad_blink.png'),
               cv2.imread(path + 'sad_talk1.png'),
               cv2.imread(path + 'sad_talk2.png'),
               cv2.imread(path + 'sad_talk3.png')]
        angry = [cv2.imread(path + 'angry.png'),
                 cv2.imread(path + 'angry_blink.png'),
                 cv2.imread(path + 'angry_talk1.png'),
                 cv2.imread(path + 'angry_talk2.png'),
                 cv2.imread(path + 'angry_talk3.png')]
        satisfied = [cv2.imread(path + 'satisfied.png'),
                     cv2.imread(path + 'satisfied_blink.png'),
                     cv2.imread(path + 'satisfied_talk1.png'),
                     cv2.imread(path + 'satisfied_talk2.png'),
                     cv2.imread(path + 'satisfied_talk3.png')]
        evil = [cv2.imread(self._pkg_path + '/src/images/evil.png'),
                cv2.imread(self._pkg_path + '/src/images/evil_blink.png'),
                cv2.imread(self._pkg_path + '/src/images/evil_talk1.png'),
                cv2.imread(self._pkg_path + '/src/images/evil_talk2.png'),
                cv2.imread(self._pkg_path + '/src/images/evil_talk3.png')]

        imgs = [neutral, eyebrows, happy, confused,
                thinking, sad, angry, satisfied, evil]

        # Images with the eyes looking to each of the four corners.
        eyes_imgs = [cv2.imread(path + 'eye_tl.png', -1),
                     cv2.imread(path + 'eye_bl.png', -1),
                     cv2.imread(path + 'eye_tr.png', -1),
                     cv2.imread(path + 'eye_br.png', -1),
                     cv2.imread(self._pkg_path + '/src/images/eye_tl.png', -1),
                     cv2.imread(self._pkg_path + '/src/images/eye_bl.png', -1),
                     cv2.imread(self._pkg_path + '/src/images/eye_tr.png', -1),
                     cv2.imread(self._pkg_path + '/src/images/eye_br.png', -1)]

        # Image will be resized relatively to the least scale, so it will not
        # be deformed.
        self._original_width = imgs[0][0].shape[1]
        self._original_height = imgs[0][0].shape[0]
        scale_y = self._window_px[1] / self._original_height
        scale_x = self._window_px[0] / self._original_width
        if scale_y < scale_x:
            scale = scale_y
        else:
            scale = scale_x

        # New dimensions for the images.
        self._image_width = int(imgs[0][0].shape[1] * scale)
        self._image_height = int(imgs[0][0].shape[0] * scale)
        new_dimensions = (self._image_width, self._image_height)

        # Resizes the images.
        for i in range(0, len(imgs)):
            for j in range(0, len(imgs[i])):
                imgs[i][j] = cv2.resize(imgs[i][j], new_dimensions,
                                        interpolation=cv2.INTER_AREA)

        new_dimensions = (int(eyes_imgs[0].shape[1] * scale),
                          int(eyes_imgs[0].shape[0] * scale))
        for i in range(0, len(eyes_imgs)):
            eyes_imgs[i] = cv2.resize(eyes_imgs[i], new_dimensions,
                                      interpolation=cv2.INTER_AREA)

        # Updates the position of the eyes in the face image.
        for i in range(0, 4):
            self._eyes_position[i] = int(
                round(self._original_eyes_position[i] * scale))

        return imgs, eyes_imgs

    def _define_eyes_point(self):
        """
        Defines the point of the eyes related to the image's top left point.
        """
        # Original image ratio.
        ratio_x = self._original_eyecenter[0] / self._original_width
        ratio_y = self._original_eyecenter[1] / self._original_height

        px_x = ratio_x * self._image_width
        px_x = int(px_x)
        px_y = ratio_y * self._image_height
        px_y = int(px_y)

        self._eyes_px = [px_x, px_y]

    def _convert_image(self, cv_image):
        """
        Converts the image in OpenCV format to a ImageTk.PhotoImage image.

        :param cv_image: image in OpenCV format to be converted.
        :return: the ImageTk.PhotoImage image.
        """
        # Changing from BGR to RGB.
        image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        # Converting to PIL format.
        image = Image.fromarray(image)
        # Creating PhotoImage.
        image = ImageTk.PhotoImage(image)

        return image

    def _face_and_voice(self):
        """
        Defines the current face image and the controls the voice.
        """
        self._now_time = time.time()

        if not self._audio[1]:
            # If there is no command for voice.
            if self._eyes_state == 0:
                # Opened eyes.
                self._image = self._images[self._expression][0].copy()
                if self._now_time - self._eyes_start_time >= self._time_open:
                    self._eyes_start_time = time.time()
                    self._eyes_state = 1
            if self._eyes_state == 1:
                # Closed eyes.
                self._image = self._images[self._expression][1].copy()
                if self._now_time - self._eyes_start_time >= self._time_closed:
                    self._eyes_start_time = time.time()
                    self._eyes_state = 0
        else:
            # If there is a command for voice.
            if path.exists(self._audio[1][0]):
                # If the audio file exists.
                if self._audio[0] == 0:
                    # If there is no audio playing, loads and starts one.
                    time.sleep(0.5)
                    self.audio_thread = Thread(target=playsound,
                                               args=[self._audio[1][0]])
                    self.audio_thread.daemon = True
                    self.audio_thread.start()
                    self._time_audio[0] = time.time()
                    self._mouth_start_time = time.time()
                    self._save_image = self._images[self._expression].copy()
                    self._speak_pub.publish(1)
                self._image = self._save_image[self._mouth_index].copy()
                if self._now_time - self._mouth_start_time >= self._time_talking:
                    # Choosing a random mouth shape to be used next.
                    mouth_options = [2, 3, 4]
                    mouth_options.remove(self._mouth_index)
                    self._mouth_index = sample(mouth_options, 1)[0]
                    self._mouth_start_time = time.time()
                self._audio[0] = 1
                if self.audio_thread:
                    if not self.audio_thread.is_alive():
                        # If the audio finished.
                        self._audio[0] = 0
                        self.audio_thread = []

                        if time.time() - self._time_audio[0] > self._time_audio[
                            1]:
                            # If the audio played more than the minimum time.
                            self._audio[1].pop(0)
                        else:
                            print("Repeated audio:")
                            print(self._audio[1][0])

                        self._time_audio[0] = 0

                        # Showing closed mouth expression between audios.
                        self._image = self._save_image[0].copy()

                        if not self._audio[1]:
                            self._speak_pub.publish(0)
            else:
                # Print error message.
                print("\033[93m {}\033[00m".format(
                    "\nERROR: Unknown voice command."))
                print('(' + self._audio[1][0] + ')')
                self._audio[0] = 0
                self._audio[1].pop(0)
                self._speak_pub.publish(-1)

        # Adds the corner eyes on the face images.
        if self._corner_eyes > 0 and self._eyes_state == 0:
            l_x = self._eyes_position[0]
            l_y = self._eyes_position[1]
            r_x = self._eyes_position[2]
            r_y = self._eyes_position[3]
            h = self._eyes_images[0].shape[0]
            w = self._eyes_images[0].shape[1]
            if self._expression != len(self._images) - 1:
                # If it is not the evil expression.
                self._image = overlay_transparent(self._image,
                                                  self._eyes_images[
                                                      self._corner_eyes - 1],
                                                  l_x, l_y)
                self._image = overlay_transparent(self._image,
                                                  self._eyes_images[
                                                      self._corner_eyes - 1],
                                                  r_x, r_y)
            else:
                # If it is the evil expression (red eyes).
                self._image = overlay_transparent(self._image,
                                                  self._eyes_images[
                                                      self._corner_eyes + 4 - 1],
                                                  l_x, l_y)
                self._image = overlay_transparent(self._image,
                                                  self._eyes_images[
                                                      self._corner_eyes + 4 - 1],
                                                  r_x, r_y)

    def _point_to_plane_on_line(self, point, plane, line):
        """
        Calculates distance between point and plane on line direction.

        :param point: pure quaternion point.
        :param plane: dual quaternion plane.
        :param line: unit pure dual quaternion Plucker line
        :return: calculated distance.
        """
        if is_pure_quaternion(point) and is_plane(plane) and is_line(line):
            if dot(line.P(), plane.P()) != 0:
                num = plane.D() - dot(point, plane.P())
                num_vec = num.vec8()
                den = dot(line.P(), plane.P())
                den_vec = den.vec8()
                d = num_vec[0] / den_vec[0]
            else:
                d = 0
            distance = abs(d)
            return distance
        else:
            point_vec = point.vec4()
            planedual_vec = plane.D().vec4()
            if 1e-13 < point_vec[0] < 1e-11 or planedual_vec[3] < 1e-11:
                point_vec[0] = 0
                point = DQ(point_vec)
                planedual_vec[3] = 0
                plane = plane.P() + DQ.E * DQ(planedual_vec)
                if dot(line.P(), plane.P()) != 0:
                    num = plane.D() - dot(point, plane.P())
                    num_vec = num.vec8()
                    den = dot(line.P(), plane.P())
                    den_vec = den.vec8()
                    d = num_vec[0] / den_vec[0]
                else:
                    d = 0
                distance = abs(d)
                # vec_distance = distance.vec8()
                return distance
            else:
                print('>>>>>> ERROR point_to_plane_on_line(): ' +
                      'The function point_to_plane_on_line() accepts only' +
                      ' point (pure quaternion), plane (unit dual' +
                      ' quaternion with real dual part) and line (unit pure dual' +
                      ' quaternion) elements.')
                print("point: " + str(point))
                print("plane: " + str(plane))
                print("line: " + str(line))
                return -1

    def _define_depth_points(self):
        """
        Defines the points to be used to create a depth for the face.
        """
        focal_point = - self._goal_point

        # Direction of the line from focal point to rotated image corner points.
        l_tl = normalize(self._x_tl.translation() - focal_point)
        l_bl = normalize(self._x_bl.translation() - focal_point)
        l_tr = normalize(self._x_tr.translation() - focal_point)
        l_br = normalize(self._x_br.translation() - focal_point)

        # Plucker line connecting focal point to each new corner point.
        line_tl = l_tl + DQ.E * (cross(focal_point, l_tl))
        line_bl = l_bl + DQ.E * (cross(focal_point, l_bl))
        line_tr = l_tr + DQ.E * (cross(focal_point, l_tr))
        line_br = l_br + DQ.E * (cross(focal_point, l_br))

        # Creating a plane behind the screen plane.
        point_on_plane = DQ([0, 0, 0.1])
        normal = normalize(focal_point - point_on_plane)
        plane = normal + DQ.E * (dot(point_on_plane, normal))

        # Calculating crossing points between the lines and the plane.
        crossing_dist_tl = self._point_to_plane_on_line(focal_point,
                                                        plane, line_tl)
        crossing_point_tl = focal_point + crossing_dist_tl * line_tl.P()
        crossing_dist_bl = self._point_to_plane_on_line(focal_point,
                                                        plane, line_bl)
        crossing_point_bl = focal_point + crossing_dist_bl * line_bl.P()
        crossing_dist_tr = self._point_to_plane_on_line(focal_point,
                                                        plane, line_tr)
        crossing_point_tr = focal_point + crossing_dist_tr * line_tr.P()
        crossing_dist_br = self._point_to_plane_on_line(focal_point,
                                                        plane, line_br)
        crossing_point_br = focal_point + crossing_dist_br * line_br.P()

        # Pixel/meters ratio.
        px_m_ratio_x = self._screen_px[0] / self._screen_m[0]
        px_m_ratio_y = self._screen_px[1] / self._screen_m[1]

        # Converting the points from meters to pixels.
        tl = [int(vec8(crossing_point_tl)[1] * px_m_ratio_x),
              int(vec8(crossing_point_tl)[2] * px_m_ratio_y)]
        bl = [int(vec8(crossing_point_bl)[1] * px_m_ratio_x),
              int(vec8(crossing_point_bl)[2] * px_m_ratio_y)]
        tr = [int(vec8(crossing_point_tr)[1] * px_m_ratio_x),
              int(vec8(crossing_point_tr)[2] * px_m_ratio_y)]
        br = [int(vec8(crossing_point_br)[1] * px_m_ratio_x),
              int(vec8(crossing_point_br)[2] * px_m_ratio_y)]

        # Pixel points with respect to the top left corner of the image.
        shift_x = int((self._window_px[0] - self._image_width) / 2)
        shift_y = int((self._window_px[1] - self._image_height) / 2)
        self._depth_points[0] = [tl[0] + self._eyes_px[0],
                                 tl[1] + self._eyes_px[1]]
        self._depth_points[1] = [bl[0] + self._eyes_px[0],
                                 bl[1] + self._eyes_px[1]]
        self._depth_points[2] = [tr[0] + self._eyes_px[0],
                                 tr[1] + self._eyes_px[1]]
        self._depth_points[3] = [br[0] + self._eyes_px[0],
                                 br[1] + self._eyes_px[1]]

    def _update_canvas(self):
        """
        Updates the canvas object in the window.
        The canvas contains the face image and lines to create depth.
        """
        self._canvas.delete("all")

        # Adding the face image.
        self._canvas.create_image(
            int((self._window_px[0] - self._image_width) / 2),
            int((self._window_px[1] - self._image_height) / 2),
            image=self._photo,
            anchor=NW)

        shift_x = int((self._window_px[0] - self._image_width) / 2)
        shift_y = int((self._window_px[1] - self._image_height) / 2)

        # Drawing the lines around the face.
        self._canvas.create_line(self._tl[0] + shift_x, self._tl[1] + shift_y,
                                 self._bl[0] + shift_x, self._bl[1] + shift_y,
                                 fill="white")
        self._canvas.create_line(self._tl[0] + shift_x, self._tl[1] + shift_y,
                                 self._tr[0] + shift_x, self._tr[1] + shift_y,
                                 fill="white")
        self._canvas.create_line(self._bl[0] + shift_x, self._bl[1] + shift_y,
                                 self._br[0] + shift_x, self._br[1] + shift_y,
                                 fill="white")
        self._canvas.create_line(self._tr[0] + shift_x, self._tr[1] + shift_y,
                                 self._br[0] + shift_x, self._br[1] + shift_y,
                                 fill="white")

        # Drawing the depth lines.
        if self._depth_points[0][0] < self._tl[0] or self._depth_points[0][1] < \
                self._tl[1]:
            self._canvas.create_line(self._tl[0] + shift_x,
                                     self._tl[1] + shift_y,
                                     self._depth_points[0][0] + shift_x,
                                     self._depth_points[0][1] + shift_y,
                                     fill="white")
        if self._depth_points[1][0] < self._bl[0] or self._depth_points[1][1] > \
                self._bl[1]:
            self._canvas.create_line(self._bl[0] + shift_x,
                                     self._bl[1] + shift_y,
                                     self._depth_points[1][0] + shift_x,
                                     self._depth_points[1][1] + shift_y,
                                     fill="white")
        if self._depth_points[2][0] > self._tr[0] or self._depth_points[2][1] < \
                self._tr[1]:
            self._canvas.create_line(self._tr[0] + shift_x,
                                     self._tr[1] + shift_y,
                                     self._depth_points[2][0] + shift_x,
                                     self._depth_points[2][1] + shift_y,
                                     fill="white")
        if self._depth_points[3][0] > self._br[0] or self._depth_points[3][1] > \
                self._br[1]:
            self._canvas.create_line(self._br[0] + shift_x,
                                     self._br[1] + shift_y,
                                     self._depth_points[3][0] + shift_x,
                                     self._depth_points[3][1] + shift_y,
                                     fill="white")

    def _define_transformation(self):
        """
        Defines the perspective transformation that need to be done with the
        image of the face in order to make the virtual agent look at a desired
        point.
        """
        if self._goal_point == self._ref_point:
            # Top left corner.
            self._final_corners[0] = [0, 0]
            # Bottom left corner.
            self._final_corners[1] = [0, self._image_height]
            # Top right corner.
            self._final_corners[2] = [self._image_width, 0]
            # Bottom right corner.
            self._final_corners[3] = [self._image_width,
                                      self._image_height]
            self._depth_points = self._final_corners.copy()
        else:
            # We use the point of the center of the eyes as origin.
            # The frame attached to it has x-axis going right, y-axis going down
            # and z-axis going inside the screen.

            # Point of the center of the eyes.
            eyes_center_point = DQ([0, 0, 0])

            # Coordinates of the goal point.
            x_goal = vec3(self._goal_point)[0]
            y_goal = vec3(self._goal_point)[1]
            z_goal = vec3(self._goal_point)[2]

            # Absolute distance from the eyes center point and the goal point.
            D_goal = vec8(norm(self._goal_point - eyes_center_point))[0]

            # Finding the rotation angles. Theta is the angle around y-axis and
            # phi is the angle around x-axis.
            theta = math.atan(-x_goal / abs(z_goal))
            phi = math.asin(y_goal / D_goal)

            # Creating the rotation quaternions ry and rx.
            ry = math.cos(theta / 2) + DQ.j * math.sin(theta / 2)
            rx = math.cos(phi / 2) + DQ.i * math.sin(phi / 2)

            # Rotated eyes center frame with z-axis pointing at goal point.
            x_rotated = normalize(rx * ry)

            # Pixel/meters ratio.
            px_m_ratio_x = self._screen_px[0] / self._screen_m[0]
            px_m_ratio_y = self._screen_px[1] / self._screen_m[1]

            # Image dimensions in meters.
            image_width_m = self._image_width / px_m_ratio_x
            image_height_m = self._image_height / px_m_ratio_y

            # Creating the dual quaternions transformations for the corner
            # points in the rotated plane (from the point of view of rotated
            # frame).
            eyes_m = [self._eyes_px[0] / px_m_ratio_x,
                      self._eyes_px[1] / px_m_ratio_y]

            p_rotated_tl = DQ([0, -eyes_m[0], -eyes_m[1], 0])
            p_rotated_bl = DQ([0, -eyes_m[0], image_height_m - eyes_m[1], 0])
            p_rotated_tr = DQ([0, image_width_m - eyes_m[0], -eyes_m[1], 0])
            p_rotated_br = DQ([0, image_width_m - eyes_m[0],
                               image_height_m - eyes_m[1], 0])

            x_rotated_tl = 1 + DQ.E * (1 / 2) * p_rotated_tl * 1
            x_rotated_bl = 1 + DQ.E * (1 / 2) * p_rotated_bl * 1
            x_rotated_tr = 1 + DQ.E * (1 / 2) * p_rotated_tr * 1
            x_rotated_br = 1 + DQ.E * (1 / 2) * p_rotated_br * 1

            # Corner points in the rotated plane from the point of view of the
            # original eyes center frame.
            self._x_tl = x_rotated * x_rotated_tl
            self._x_bl = x_rotated * x_rotated_bl
            self._x_tr = x_rotated * x_rotated_tr
            self._x_br = x_rotated * x_rotated_br

            # Directions of the lines from the reference point to the rotated
            # corner points.
            l_tl = normalize(translation(self._x_tl) - self._ref_point)
            l_bl = normalize(translation(self._x_bl) - self._ref_point)
            l_tr = normalize(translation(self._x_tr) - self._ref_point)
            l_br = normalize(translation(self._x_br) - self._ref_point)

            # Plucker lines connecting reference point to each of the rotated
            # corner points.
            line_tl = l_tl + DQ.E * cross(self._ref_point, l_tl)
            line_bl = l_bl + DQ.E * cross(self._ref_point, l_bl)
            line_tr = l_tr + DQ.E * cross(self._ref_point, l_tr)
            line_br = l_br + DQ.E * cross(self._ref_point, l_br)

            # Creating the plane of the screen.
            reference_normal = normalize(self._ref_point - eyes_center_point)
            reference_plane = reference_normal + \
                              DQ.E * dot(eyes_center_point, reference_normal)

            # Calculating crossing points between the lines and the screen
            # plane. These crossing points will be the new corner points in the
            # screen.
            crossing_dist_tl = self._point_to_plane_on_line(self._ref_point,
                                                            reference_plane,
                                                            line_tl)
            crossing_point_tl = self._ref_point + crossing_dist_tl * line_tl.P()
            crossing_dist_bl = self._point_to_plane_on_line(self._ref_point,
                                                            reference_plane,
                                                            line_bl)
            crossing_point_bl = self._ref_point + crossing_dist_bl * line_bl.P()
            crossing_dist_tr = self._point_to_plane_on_line(self._ref_point,
                                                            reference_plane,
                                                            line_tr)
            crossing_point_tr = self._ref_point + crossing_dist_tr * line_tr.P()
            crossing_dist_br = self._point_to_plane_on_line(self._ref_point,
                                                            reference_plane,
                                                            line_br)
            crossing_point_br = self._ref_point + crossing_dist_br * line_br.P()

            # Converting the points from meters to pixels.
            tl = [int(vec8(crossing_point_tl)[1] * px_m_ratio_x),
                  int(vec8(crossing_point_tl)[2] * px_m_ratio_y)]
            bl = [int(vec8(crossing_point_bl)[1] * px_m_ratio_x),
                  int(vec8(crossing_point_bl)[2] * px_m_ratio_y)]
            tr = [int(vec8(crossing_point_tr)[1] * px_m_ratio_x),
                  int(vec8(crossing_point_tr)[2] * px_m_ratio_y)]
            br = [int(vec8(crossing_point_br)[1] * px_m_ratio_x),
                  int(vec8(crossing_point_br)[2] * px_m_ratio_y)]

            # Pixel points with respect to the top left corner of the image.
            self._final_corners[0] = [tl[0] + self._eyes_px[0],
                                      tl[1] + self._eyes_px[1]]
            self._final_corners[1] = [bl[0] + self._eyes_px[0],
                                      bl[1] + self._eyes_px[1]]
            self._final_corners[2] = [tr[0] + self._eyes_px[0],
                                      tr[1] + self._eyes_px[1]]
            self._final_corners[3] = [br[0] + self._eyes_px[0],
                                      br[1] + self._eyes_px[1]]

            self._define_depth_points()

    def _gaze(self):
        """
        Updates the gaze of the virtual agent.
        """
        if self._x0_goal != DQ([1]):
            # If a gaze command was already received.

            # Eyes center with respect to marker 0.
            x0_eyes = self._x0_marker * self._x_marker_eyes
            # Goal point with respect to the eyes.
            self._x_eyes_goal = x0_eyes.conj() * self._x0_goal

        # Extracting the translation.
        p_eyes_goal_vec = vec4(translation(self._x_eyes_goal))
        p_eyes_goal = DQ(p_eyes_goal_vec[1:4])
        if self._goal_point != p_eyes_goal:
            # If goal point changed, update the variables.
            self._goal_point = p_eyes_goal
            self._define_transformation()

            start_tl = self._tl
            start_bl = self._bl
            start_tr = self._tr
            start_br = self._br
            self._start_corners = np.float32([start_tl, start_bl,
                                              start_tr, start_br])

            # _iterations[0] = total number of iterations.
            # _iterations[1] = current iteration.
            self._iterations = [5, 0]

        # Calculates the new corner points.
        remaining_iterations = self._iterations[0] - self._iterations[1] + 1
        step_tl = [int(
            (self._final_corners[0][0] - self._tl[0]) / remaining_iterations),
            int((self._final_corners[0][1] - self._tl[
                1]) / remaining_iterations)]
        step_bl = [int(
            (self._final_corners[1][0] - self._bl[0]) / remaining_iterations),
            int((self._final_corners[1][1] - self._bl[
                1]) / remaining_iterations)]
        step_tr = [int(
            (self._final_corners[2][0] - self._tr[0]) / remaining_iterations),
            int((self._final_corners[2][1] - self._tr[
                1]) / remaining_iterations)]
        step_br = [int(
            (self._final_corners[3][0] - self._br[0]) / remaining_iterations),
            int((self._final_corners[3][1] - self._br[
                1]) / remaining_iterations)]

        self._tl[0] = self._tl[0] + step_tl[0]
        self._tl[1] = self._tl[1] + step_tl[1]
        self._bl[0] = self._bl[0] + step_bl[0]
        self._bl[1] = self._bl[1] + step_bl[1]
        self._tr[0] = self._tr[0] + step_tr[0]
        self._tr[1] = self._tr[1] + step_tr[1]
        self._br[0] = self._br[0] + step_br[0]
        self._br[1] = self._br[1] + step_br[1]

        self._new_corners = np.float32([self._tl, self._bl, self._tr, self._br])

        # Executes the transformation.
        matrix = cv2.getPerspectiveTransform(self._ref_corners,
                                             self._start_corners)
        self._image = cv2.warpPerspective(self._image, matrix,
                                          (self._image_width,
                                           self._image_height))
        matrix = cv2.getPerspectiveTransform(self._start_corners,
                                             self._new_corners)
        self._image = cv2.warpPerspective(self._image, matrix,
                                          (self._image_width,
                                           self._image_height))
        if self._iterations[1] < self._iterations[0]:
            self._iterations[1] = self._iterations[1] + 1

    def _expression_callback(self, msg):
        """
        Callback method to get the facial expression command.
        Gets the command and updates the _expression attribute.

        :param msg: a std_msgs/String message with the expression command.
        :return:
        """
        exp = msg.data
        if exp == 'neutral':
            self._expression = 0
        elif exp == 'eyebrows':
            self._expression = 1
        elif exp == 'happy':
            self._expression = 2
        elif exp == 'confused':
            self._expression = 3
        elif exp == 'thinking':
            self._expression = 4
        elif exp == 'sad':
            self._expression = 5
        elif exp == 'angry':
            self._expression = 6
        elif exp == 'satisfied':
            self._expression = 7
        elif exp == 'evil':
            self._expression = 8
        else:
            # Print error message in yellow.
            print("\033[93m {}\033[00m".format(
                "\nERROR: Unknown expression command."))
            print('(' + exp + ')')

    def _voice_callback(self, msg):
        """
        Callback method to get the voice command.
        Gets the command and updates the _audio attribute.

        :param msg: a std_msgs/String message with the name of the audio file
        to be executed.
        :return:
        """
        if msg.data != '0':
            file = self._pkg_path + '/src/audio-files/' + self._current_va + \
                   '/' + msg.data + '.mp3'
            if len(self._audio[1]) == 0:
                self._audio[1].append(file)
            else:
                if file != self._audio[1][len(self._audio[1]) - 1]:
                    self._audio[1].append(file)
        else:
            # If the message is '0', initializes audio variable.
            self._audio = [0, []]

    def _gaze_callback(self, msg):
        """
        Callback method to get the gaze command.
        Gets the command and updates the _goal_point attribute.

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

        if x == 0 and y == 0 and z == 0:
            # If position is zero, looks to the reference point.
            self._ref_point = DQ([0, 0, -0.5])
            self._x_eyes_ref = 1 + DQ.E * 0.5 * self._ref_point * 1
            self._x0_goal = self._x0_marker * self._x_marker_eyes * self._x_eyes_ref
            self._corner_eyes = 0
        elif x == 1 and y == 1 and z == 1:
            # Looks to the top-left corner.
            self._ref_point = DQ([0, 0, -0.5])
            self._x_eyes_ref = 1 + DQ.E * 0.5 * self._ref_point * 1
            point = self._ref_point + DQ([-0.1, -0.1, 0])
            x_eyes_point = normalize(1 + DQ.E * 0.5 * point * 1)
            self._x0_goal = self._x0_marker * self._x_marker_eyes * x_eyes_point
            self._corner_eyes = 1
        elif x == 2 and y == 2 and z == 2:
            # Looks to the bottom-left corner.
            self._ref_point = DQ([0, 0, -0.5])
            self._x_eyes_ref = 1 + DQ.E * 0.5 * self._ref_point * 1
            point = self._ref_point + DQ([-0.1, 0.1, 0])
            x_eyes_point = normalize(1 + DQ.E * 0.5 * point * 1)
            self._x0_goal = self._x0_marker * self._x_marker_eyes * x_eyes_point
            self._corner_eyes = 2
        elif x == 3 and y == 3 and z == 3:
            # Looks to the top-right corner.
            self._ref_point = DQ([0, 0, -0.5])
            self._x_eyes_ref = 1 + DQ.E * 0.5 * self._ref_point * 1
            point = self._ref_point + DQ([0.1, -0.1, 0])
            x_eyes_point = normalize(1 + DQ.E * 0.5 * point * 1)
            self._x0_goal = self._x0_marker * self._x_marker_eyes * x_eyes_point
            self._corner_eyes = 3
        elif x == 4 and y == 4 and z == 4:
            # Looks to the bottom-right corner.
            self._ref_point = DQ([0, 0, -0.5])
            self._x_eyes_ref = 1 + DQ.E * 0.5 * self._ref_point * 1
            point = self._ref_point + DQ([0.1, 0.1, 0])
            x_eyes_point = normalize(1 + DQ.E * 0.5 * point * 1)
            self._x0_goal = self._x0_marker * self._x_marker_eyes * x_eyes_point
            self._corner_eyes = 4
        else:
            r = DQ([w, wx, wy, wz])
            p = DQ([0, x, y, z])
            self._x0_goal = normalize(r + DQ.E * 0.5 * p * r)
            self._corner_eyes = 0

            # Eyes center with respect to marker 0.
            x0_eyes = self._x0_marker * self._x_marker_eyes
            # Goal point with respect to the eyes.
            self._x_eyes_goal = x0_eyes.conj() * self._x0_goal
            self._ref_point = DQ([0, 0, vec3(translation(self._x_eyes_goal))[2]])
            self._x_eyes_ref = 1 + DQ.E * 0.5 * self._ref_point * 1

    def _marker_callback(self, msg):
        """
        Callback method to get the pose of the virtual agent's marker.

        :param msg:
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
        self._x0_marker = normalize(r + DQ.E * 0.5 * p * r)

    def update_all(self):
        """
        Updates the window and the virtual agent.
        This method should be called inside a while loop in the main script.
        """
        # Updating the virtual agent.
        self._face_and_voice()
        self._gaze()
        self._photo = self._convert_image(self._image)
        self._update_canvas()
        self._window.title(self._current_va.upper())

        # Updating the window.
        self._window.update_idletasks()
        #self._window.update()

    def destroy_all(self):
        """
        Destroys the main window.

        :return:
        """
        self._window.destroy()

    def set_virtual_agent(self, agent):
        """
        Updates the current images and the title of the window for the selected
        virtual agent.

        :param agent: the virtual agent.
        """
        if type(agent) is String:
            # If the function parameter is a std_msgs/String message.
            agent = agent.data

        if agent == self.Luna:
            self._images = self._luna_images.copy()
            self._eyes_images = self._luna_eyes.copy()
        if agent == self.Sofia:
            self._images = self._sofia_images.copy()
            self._eyes_images = self._sofia_eyes.copy()

        self._image = self._images[0][0].copy()
        self._current_va = agent


    def __init__(self, marker_topic, x_marker_tl, first_plan,
                 screen_m, dimensions=0, position=0):
        """
        Constructor method of the class

        :param marker_topic: the name of the topic related to the VA's screen.
        :param x_marker_tl: transformation from marker to the screen's top-left point.
        :param first_plan: indicates if the VA's window should be always in the first plan or not.
        :param screen_m: screen's dimensions in meters.
        :param dimensions: relative dimensions for the VA's window (reduction scale).
        :param position: relative position of the VA's.
        """

        # Getting the path for the ROS package.
        rospack = rospkg.RosPack()
        self._pkg_path = rospack.get_path('virtual-agent')

        # Code for black color.
        self._black = "#000000"

        # Creating and configuring the window.
        self._window = Tk()
        self._screen_px = (self._window.winfo_screenwidth(),
                           self._window.winfo_screenheight())
        self._screen_m = screen_m

        if first_plan == 1:
            # If the window should be always in the first plan.
            self._window.wm_attributes("-topmost", True)

        # The parameter 'dimensions' scales the window relative to the screen
        # dimensions.
        if dimensions == 0:
            self._window_px = self._screen_px
            self._window.attributes('-fullscreen', True)
        else:
            width = int(self._screen_px[0] / dimensions[0])
            height = int(self._screen_px[1] / dimensions[1])
            self._window_px = (width, height)

        # The parameter 'position' defines the window position relative to the
        # screen dimensions.
        if position == 0:
            self._window_position = [0, 0]
        else:
            x = int(self._screen_px[0] / position[0])
            y = int(self._screen_px[1] / position[1])
            self._window_position = (x, y)
        window_geometry = (self._window_px[0], self._window_px[1],
                           self._window_position[0], self._window_position[1])

        self._window.geometry("%dx%d+%d+%d" % window_geometry)
        self._window.configure(background=self._black)

        # Names of the virtual agents.
        self.Luna = 'luna'
        self.Sofia = 'sofia'

        # Position of the eyes in the face image.
        # [left-eyeX, left-eyeY, right-eyeX, right-eyeY]
        self._original_eyes_position = [143.844, 200.614, 645.649, 200.614]
        self._eyes_position = self._original_eyes_position.copy()

        # Configuring images of each virtual agent.
        [self._luna_images, self._luna_eyes] = self._configure_images(self.Luna)
        [self._sofia_images, self._sofia_eyes] = self._configure_images(
            self.Sofia)

        # Setting points used for transformation.
        self._original_eyecenter = [511, 318]
        self._define_eyes_point()

        self.set_virtual_agent(self.Luna)

        self._canvas = Canvas(self._window, width=self._window_px[0],
                              height=self._window_px[1], background=self._black,
                              highlightbackground=self._black)
        self._canvas.place(x=0, y=0)

        # Times for the animation of the face.
        self._time_open = 4  # Blink each 4 seconds.
        self._time_closed = 0.2  # Blinking lasts 200 ms.
        self._time_talking = 0.3  # Talking mouth changes each 300 ms.

        # State of the eye (0 if opened, 1 if closed).
        self._eyes_state = 0

        # Indicates if the eyes should be changed.
        # 0: original eyes.
        # 1: top-left corner eyes.
        # 2: bottom-left corner eyes.
        # 3: top-right corner eyes.
        # 4: bottom-right corner eyes.
        self._corner_eyes = 0

        # Index of the mouth shape being used.
        self._mouth_index = 2

        # Time variables to open and close eyes and change the mouth shape.
        self._now_time = 0
        self._eyes_start_time = 0
        self._mouth_start_time = 0

        # Audio data:
        # audio[0] = 1 if playing an audio, 0 if not.
        # audio[1] = audio queue.
        self._audio = [0, []]

        # To check if the audio was executed or not.
        # time_audio[0] = time when the audio started playing.
        # time_audio[1] = minimum playing time.
        self._time_audio = [0, 1]

        self._audio_thread = []

        # Index of the current expression.
        self._expression = 0

        # Reference point in front of the virtual agent.
        self._ref_point = DQ([0, 0, -0.50])
        self._x_eyes_ref = 1 + DQ.E * 0.5 * self._ref_point * 1

        # Goal point to where the virtual agent should look.
        self._x0_goal = DQ([1])
        self._goal_point = self._ref_point
        self._x_eyes_goal = self._x_eyes_ref

        # Initializing corners points variables.
        # Points are with respect to the images' top left corner.
        self._tl = [0, 0]
        self._bl = [0, self._image_height]
        self._tr = [self._image_width, 0]
        self._br = [self._image_width, self._image_height]
        self._ref_corners = np.float32([self._tl, self._bl, self._tr, self._br])
        self._start_corners = np.float32(
            [self._tl, self._bl, self._tr, self._br])
        self._final_corners = [self._tl, self._bl, self._tr, self._br]
        self._depth_points = [self._tl, self._bl, self._tr, self._br]

        # Iterations: [total number, counter]
        self._iterations = [5, 0]

        # Pixel/meters ratio.
        px_m_ratio_x = self._screen_px[0] / self._screen_m[0]
        px_m_ratio_y = self._screen_px[1] / self._screen_m[1]

        # Transformation from screen's top-left point to the center of the eyes.
        shift_x = int((self._window_px[0] - self._image_width) / 2)
        shift_y = int((self._window_px[1] - self._image_height) / 2)
        eyes_x = self._window_position[0] + shift_x + self._eyes_px[0]
        eyes_x = eyes_x / px_m_ratio_x
        eyes_y = self._window_position[1] + shift_y + self._eyes_px[1]
        eyes_y = eyes_y / px_m_ratio_y
        p_tl_eyes = DQ([eyes_x, eyes_y, 0])
        x_tl_eyes = normalize(1 + DQ.E * 0.5 * p_tl_eyes * 1)

        # Transformation from marker to the original eyes center in the screen.
        self._x_marker_eyes = x_marker_tl * x_tl_eyes

        # Transformation from reference marker to the marker related to the
        # virtual agent.
        self._x0_marker = DQ([1])

        # Publisher to inform if the virtual agent is speaking (1) or not (0).
        self._speak_pub = rospy.Publisher('/va_speaking', Int16,
                                          queue_size=1, latch=True)

        # Subscriber for the virtual agent's marker.
        rospy.Subscriber(marker_topic, Odometry, self._marker_callback)

        # Subscriber to change the current virtual agent.
        rospy.Subscriber('/virtual_agent', String, self.set_virtual_agent)

        # Subscribers for the commands of expression, voice and gaze.
        rospy.Subscriber('/expression', String, self._expression_callback)
        rospy.Subscriber('/voice', String, self._voice_callback)
        rospy.Subscriber('/va_gaze', Odometry, self._gaze_callback)


