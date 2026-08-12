#!/usr/bin/env python3.6
from Password import *
import rospy
import time


def main():
    rospy.init_node('password_screen')
    p = Password((1000, 500))
    p.set_password([p.blue, p.yellow, p.white, p.black])
    p.accepting_indication = 1

    while not rospy.is_shutdown():
        p.update_all()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

