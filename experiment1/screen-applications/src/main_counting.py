#!/usr/bin/env python3.6
from Counting import *


def main():
    rospy.init_node('counting_screen')
    c = Counting((1280, 720))
    c.settings([2, 1, 2, 5, 6], 1)
    #c.update_all()
    #c.fill_entries([2, 0, -1, -1])

    while not rospy.is_shutdown():
        c.update_all()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
