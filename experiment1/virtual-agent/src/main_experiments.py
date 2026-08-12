#!/usr/bin/env python3.6
import rospy
from std_msgs.msg import Int16
from dqrobotics import *
from VirtualAgent import *
from tkinter import *


class Data:
    def phase_callback(self, msg):
        self.phase = msg.data

    def counting_callback(self, msg):
        self.status_counting = msg.data

    def screen_callback(self, msg):
        self.screen = msg.data

    def __init__(self):
        self.phase = -1
        self.status_counting = -10
        self.screen = 0

        rospy.Subscriber('/phase', Int16, self.phase_callback)
        rospy.Subscriber('/status_counting', Int16, self.counting_callback)
        rospy.Subscriber('/screen_commands', Int16, self.screen_callback)


def main():
    rospy.init_node('agent_virtual_node')

    # Screen dimensions.
    screen_width = rospy.get_param("~screen_width")
    screen_height = rospy.get_param("~screen_height")
    screen_m = (screen_width, screen_height)

    # Reduction scale for the virtual agent's window.
    width_scale = rospy.get_param("~width_scale")
    height_scale = rospy.get_param("~height_scale")
    w_dim = (width_scale, height_scale)

    # Position of the virtual agent's window relative to the screen dimensions.
    window_x = rospy.get_param("~window_x")
    window_y = rospy.get_param("~window_y")
    w_pos = (window_x, window_y)

    if w_dim == (1, 1) or w_pos == (0, 0):
        fullscreen = 1
    else:
        fullscreen = 0

    # Parameter to indicate it the window should be always in first plan.
    first_plan = rospy.get_param("~first_plan")

    # Parameter to indicate in which phase the virtual agent should run.
    run_phase = rospy.get_param("~phase")

    # Marker topic.
    topic = rospy.get_param("~topic")

    # Transformation from marker to the top-left point of the screen.
    p_x = rospy.get_param("~marker_tl_x")
    p_y = rospy.get_param("~marker_tl_y")
    p_z = rospy.get_param("~marker_tl_z")
    p_marker_tl = DQ([p_x, p_y, p_z])
    phi_x = -math.pi / 2
    r_marker_tl = math.cos(phi_x / 2) + DQ.i * math.sin(phi_x / 2)
    x_marker_tl = normalize(r_marker_tl + DQ.E * 0.5 * p_marker_tl * r_marker_tl)

    # Blank window for when the virtual agent of phase 1 is not running.
    blank_window = Tk()
    blank_window.attributes('-fullscreen', True)
    blank_window.configure(background="#000000")

    d = Data()

    va = []
    while not rospy.is_shutdown():
        p = d.phase
        sc = d.status_counting

        # If the virtual agent is for phase 1.
        if run_phase == 1:
            if p == 2 and sc == -1:
                if va:
                    va.destroy_all()
                    del va
                    va = []
                    blank_window = Tk()
                    blank_window.attributes('-fullscreen', True)
                    blank_window.configure(background="#000000")
                d.phase = -1
                d.status_counting = -10
                p = -1
            if p == 1:
                blank_window.destroy()
                blank_window = []
                if fullscreen == 1:
                    va = VirtualAgent(topic, x_marker_tl, first_plan, screen_m)
                else:
                    va = VirtualAgent(topic, x_marker_tl, first_plan,
                                      screen_m, w_dim, w_pos)
                d.status_counting = -10
                d.phase = 0

            if va:
                va.update_all()

            if blank_window:
                blank_window.update_idletasks()

        # If the virtual agent is for phase 2.
        if run_phase == 2:
            if blank_window:
                blank_window.destroy()
                blank_window = []
            if p == 1:
                if va:
                    va.destroy_all()
                    del va
                    va = []
                d.phase = -1
                d.status_counting = -10
                p = -1
            if p == 2 and sc == -1:
                if fullscreen == 1:
                    va = VirtualAgent(topic, x_marker_tl, first_plan, screen_m)
                else:
                    va = VirtualAgent(topic, x_marker_tl, first_plan,
                                      screen_m, w_dim, w_pos)
                d.phase = 0

            if 2 < d.screen < 10:
                if va:
                    va.destroy_all()
                    del va
                    va = []
                if blank_window:
                    blank_window.destroy()
                    blank_window = []

            if va:
                va.update_all()

            if blank_window:
                blank_window.update_idletasks()

    # Destroying classes objects.
    del d
    if va:
        del va


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
