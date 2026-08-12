#!/usr/bin/env python3.6
import rospy
from std_msgs.msg import Int16
from VirtualAgent import *
from dqrobotics import *


def phase_callback(msg, phase):
    phase[0] = msg.data


def counting_callback(msg, status_counting):
    status_counting[0] = msg.data


def main():
    rospy.init_node('virtual_agent_phase1')

    phase = [-1]
    rospy.Subscriber('/phase', Int16, phase_callback, phase)

    status_counting = [-10]
    rospy.Subscriber('/status_counting', Int16,
                     counting_callback, status_counting)

    screen_m = (0.31, 0.175)

    topic = 'x05'

    # Transformation from marker to the top-left point of the screen.
    p_marker_tl = DQ([0.104, 0.155, 0.21])
    phi_x = -math.pi / 2
    phi_y = math.pi / 2
    r_marker_tl_x = math.cos(phi_x / 2) + DQ.i * math.sin(phi_x / 2)
    r_marker_tl_y = math.cos(phi_y / 2) + DQ.j * math.sin(phi_y / 2)
    r_marker_tl = r_marker_tl_x * r_marker_tl_y
    x_marker_tl = normalize(r_marker_tl + DQ.E * 0.5 * p_marker_tl * r_marker_tl)

    while not rospy.is_shutdown():
        p = phase[0]
        sc = status_counting[0]
        if p == 2 and sc == -1:
            va.destroy_all()
            phase[0] = -1
            status_counting[0] = -10
            p = -1
        if p == 1:
            va = VirtualAgent(topic, x_marker_tl, 0, screen_m)
            status_counting[0] = -10
            phase[0] = 0

        if p != -1:
            va.update_all()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

