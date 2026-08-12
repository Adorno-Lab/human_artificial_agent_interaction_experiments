#!/usr/bin/env python3.6
import rospy
import roslib
import rospkg
import subprocess
import signal
from std_msgs.msg import Int16
import logging
from imp import reload
from datetime import datetime
import sys


class Data:
    def tracker_callback(self, msg):
        self.tracker = msg.data

    def __init__(self):
        self.tracker = -1


def main():
    rospy.init_node('tracker_manager')

    # Getting the path for the ROS package.
    rospack = rospkg.RosPack()
    pkg_path = rospack.get_path('tracker')

    d = Data()

    # Subscriber for the commands to run (1) or stop (0) the tracker.
    rospy.Subscriber('/tracker_commands', Int16, d.tracker_callback)

    process = []

    # Creating and configuring the log file.
    log_file = reload(logging)
    log_path = pkg_path + "/log/"
    log_name = datetime.now().strftime("%d.%m.%Y_%H:%M:%S") + '.log'
    format_text = '%(asctime)s %(message)s'
    date_format = '%d/%m/%Y %H:%M:%S'
    log_file.basicConfig(filename=log_path + log_name, format=format_text,
                         datefmt=date_format, level=logging.INFO)

    while not rospy.is_shutdown():
        if d.tracker == 1:
            if not process:
                # Writing the command.
                command = ["roslaunch", "tracker", "launch_tracker.launch"]

                # Starting the subprocess.
                log_file.info('%s', "Starting tracker.")
                try:
                    process = subprocess.Popen(command)
                except:
                    log_file.info('%s:%s', "Error when starting tracker",
                                  sys.exc_info())
                    raise

                d.tracker = -1
        if d.tracker == 0:
            # Sending exit signal to the subprocess.
            if process:
                log_file.info('%s', "Ending tracker.")
                try:
                    process.send_signal(signal.SIGTERM)
                except:
                    log_file.info('%s:%s', "Error when finishing tracker",
                                  sys.exc_info())
                    raise

                d.tracker = -1
                process = []

    # Destroying Data class object.
    del d


if __name__ == '__main__':
    main()
