#!/usr/bin/env python3.6
import rospy
from std_msgs.msg import Int16
from dqrobotics import *
from VirtualAgent import *
from tkinter import *


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

    if fullscreen == 1:
        va = VirtualAgent(topic, x_marker_tl, first_plan, screen_m)
    else:
        va = VirtualAgent(topic, x_marker_tl, first_plan,
                          screen_m, w_dim, w_pos)

    while not rospy.is_shutdown():
        va.update_all()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
