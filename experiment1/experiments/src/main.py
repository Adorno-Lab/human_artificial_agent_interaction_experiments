#!/usr/bin/env python3.6
import rospy
import roslib
import rospkg
import time
import logging
from imp import reload
from datetime import datetime
import numpy as np
from random import sample
from std_msgs.msg import Int16, String, Float32, Int16MultiArray
from geometry_msgs.msg import Point
from include.ExPhaseOne import *
from include.ExPhaseTwo import *
from include.ExImPhaseOne import *
from include.ExImPhaseTwo import *
from include.DataHandler import *
import ex_ex_config
import exim_exim_config
import signal


class Data:
    def start_callback(self, msg):
        """
        Callback function for the start command.

        :param msg: an std_msgs/Int16 message.
        :return:
        """
        self.start = msg.data

    def string_to_list(self, s, separator, type):
        """
        Converts a string in a numerical list.

        :param s: a string to be converted.
        :param separator: the separator character(s) in the string.
        :param type: a string to indicate if the elements are "int" or "float".
        :return: a numerical list.
        """
        s = s[1:len(s)-1]
        s = s.split(separator)
        list = []
        for i in s:
            if type == "int":
                list.append(int(i))
            if type == "float":
                list.append(float(i))

        return list

    def read_shutdown_log(self):
        """
        Reads the log file generated on shutdown and updates the class arguments.

        :return:
        """
        log_file = open(self.pkg_path + "/log/on_shutdown.log", "r")
        lines = log_file.read().splitlines()

        self.phase = int(lines[0][7:len(lines[0])])
        self.config = int(lines[1][8:len(lines[1])])
        self.config_name = lines[2][13:len(lines[2])]

        self.password = lines[3][10:len(lines[3])]
        if self.password == "-1":
            self.password = int(self.password)
        else:
            self.password = self.string_to_list(self.password, ", ", "int")

        self.password_status = int(lines[4][13:len(lines[4])])

        self.virtual_agent = lines[5][15:len(lines[5])]
        self.images_set = int(lines[6][12:len(lines[6])])

        self.state = lines[7][11:len(lines[7])]
        if self.state == "-1":
            self.state = int(self.state)
        else:
            self.state = self.string_to_list(self.state, ", ", "float")

        self.timers = lines[8][8:len(lines[8])]
        if self.timers == "-1":
            self.timers = int(self.timers)
        else:
            self.timers = self.string_to_list(self.timers, ", ", "float")

        self.password_errors = int(lines[9][13:len(lines[9])])
        self.interaction_time = float(lines[10][10:len(lines[10])])
        self.start_time = float(lines[11][12:len(lines[11])])

        self.entries = lines[12][9:len(lines[12])]
        if self.entries == "-1":
            self.entries = int(self.entries)
        else:
            self.entries = self.string_to_list(self.entries, ", ", "int")

        self.shutdown_time = float(lines[13][15:len(lines[13])])

    def shutdown(self):
        """
        Function executed on node shutdown. It stops the record nodes and writes
        the shutdown log file.

        :return:
        """
        # Stopping record nodes.
        self.record_pub.publish(0)

        # Writing shutdown log file.
        log_file = open(self.pkg_path + "/log/on_shutdown.log", "w")
        log_file.write("PHASE: " + str(self.phase) + "\n")
        log_file.write("CONFIG: " + str(self.config) + "\n")
        log_file.write("CONFIG NAME: " + str(self.config_name) + "\n")
        log_file.write("PASSWORD: " + str(self.password) + "\n")
        log_file.write("PASS STATUS: " + str(self.password_status) + "\n")
        log_file.write("VIRTUAL AGENT: " + str(self.virtual_agent) + "\n")
        log_file.write("IMAGES SET: " + str(self.images_set) + "\n")
        log_file.write("NET STATE: " + str(self.state) + "\n")
        log_file.write("TIMERS: " + str(self.timers) + "\n")
        log_file.write("PASS ERRORS: " + str(self.password_errors) + "\n")
        log_file.write("INT TIME: " + str(self.interaction_time) + "\n")
        log_file.write("START TIME: " + str(self.start_time) + "\n")
        log_file.write("ENTRIES: " + str(self.entries) + "\n")
        log_file.write("SHUTDOWN_TIME: " + str(time.time()) + "\n")
        log_file.close()
        print("Log file written.")

    def __init__(self):
        # Package path.
        self.pkg_path = -1

        # Current phase.
        self.phase = -1

        # Current configuration.
        self.config = -1

        # Current configuration name.
        self.config_name = -1

        # Current password.
        self.password = -1

        # Current status of the password application.
        self.password_status = -1

        # Current virtual agent.
        self.virtual_agent = -1

        # Current counting images set.
        self.images_set = -1

        # Current net state.
        self.state = -1

        # Current times.
        self.timers = -1

        # Current number of password errors.
        self.password_errors = 0

        # Interaction time.
        self.interaction_time = 0

        # Start time.
        self.start_time = 0

        # Added entries in counting phase.
        self.entries = -1

        # Time when node was shutdown.
        self.shutdown_time = -1

        # Publisher for the record nodes.
        self.record_pub = rospy.Publisher('/record', Int16,
                                          queue_size=1, latch=True)

        # Publisher for the status of the counting application.
        self.sc_pub = rospy.Publisher('/status_counting', Int16,
                                      queue_size=1, latch=True)

        self.start = 0
        rospy.Subscriber('/start', Int16, self.start_callback)


def define_password():
    """
    Defines a random password.

    :return: a list with the password.
    """
    options = [1, 2, 3, 4]

    password = []
    password.append(sample(options, 1)[0])
    options.remove(password[0])
    password.append(sample(options, 1)[0])
    options.append(password[0])
    options.remove(password[1])
    password.append(sample(options, 1)[0])
    options.append(password[1])
    options.remove(password[2])
    password.append(sample(options, 1)[0])

    return password


def define_net(config, phase):
    """
    Defines the Petri net for the given configuration and phase.

    :param config: communication configuration.
    :param phase: phase.
    :return: the selected net object.
    """
    if config == 0:
        if phase == 1:
            print("EX/EX - phase1")
            return ExPhaseOne()
        if phase == 2:
            print("EX/EX - phase2")
            return ExPhaseTwo()
    if config == 1:
        if phase == 1:
            print("EXIM/EXIM - phase1")
            return ExImPhaseOne()
        if phase == 2:
            print("EXIM/EXIM - phase2")
            return ExImPhaseTwo()


def main():
    rospy.init_node('petri_net_node')

    d = Data()
    dh = DataHandler()

    # Getting the path for the ROS package.
    rospack = rospkg.RosPack()
    d.pkg_path = rospack.get_path('experiments')

    # Getting parameter from the launch file.
    recover = rospy.get_param("~recover")

    if recover == 1:
        # Recovering data from the shutdown log file.
        d.read_shutdown_log()

    # Defining phase.
    if recover == 1 and d.phase != -1:
        dh.phase = d.phase
    else:
        dh.phase = 1

    # Defining order for the configurations.
    # Configuration: 0 - EX/EX
    #                1 - EXIM/EXIM
    config_names = ['EX/EX', 'EXIM/EXIM']
    config = sample(range(0, 2), 2)
    if recover == 1 and d.config != -1:
        # Updating order using recovered data.
        if config_names[config[d.config]] != d.config_name:
            config.reverse()

    # Defining configuration and configuration index.
    if recover == 1 and d.config > 0:
        c = d.config
        if c == len(config) - 1:
            # If it is the last configuration.
            dh.configuration = 2
        else:
            # If it is not the last configuration.
            dh.configuration = 1
    else:
        c = 0
        dh.configuration = c

    # Defining the virtual agents order.
    va_options = ['luna', 'sofia']
    va_order = []
    for i in sample(range(0, 2), 2):
        va_order.append(va_options[i])
    if recover == 1 and d.virtual_agent != "-1":
        # Updating order using recovered data.
        if va_order[d.config] != d.virtual_agent:
            va_order.reverse()

    # Defining order for the images of the counting phase.
    images_order = sample(range(1, 3), 2)
    images_set = images_order.copy()
    if recover == 1 and d.images_set != -1:
        # Updating order using recovered data.
        if d.phase == 1:
            # If it is in phase 1 and the image_set is defined, the stored value
            # is from the previous configuration.
            index = d.config - 1
        else:
            index = d.config
        if images_set[index] != d.images_set:
            images_set.reverse()

    # The correct values for each images set.
    # Images set / Values
    # 1          / [7, 6, 8, 5]
    # 2          / [8, 5, 6, 7]
    counting = [[7, 6, 8, 5], [8, 5, 6, 7]]

    # Defining the order of the counting values according to the images order.
    counting_values = []
    for i in images_set:
        counting_values.append(counting[i - 1])

    if recover == 1 and d.phase == 1:
        # If recovering data, there is no need to wait for the start command.
        d.start = 1

    # Publisher to inform the end of all the configurations.
    end_pub = rospy.Publisher('/end', Int16, queue_size=1, latch=True)

    net = []

    # Creating and configuring the log file.
    dh.log_file = reload(logging)
    log_path = d.pkg_path + "/log/"
    log_name = datetime.now().strftime("%d.%m.%Y_%H:%M:%S") + '.log'
    format_text = '%(asctime)s %(message)s'
    date_format = '%d/%m/%Y %H:%M:%S'
    dh.log_file.basicConfig(filename=log_path + log_name,
                            format=format_text,
                            datefmt=date_format,
                            level=logging.INFO)
    dh.log_file.disable(dh.log_file.WARNING)

    rospy.on_shutdown(d.shutdown)
    rate = rospy.Rate(50)

    while not rospy.is_shutdown():

        if dh.phase == 1:
            # First phase (password).
            d.phase = dh.phase
            d.config = c
            d.config_name = config_names[config[c]]

            if dh.configuration == 0:
                # If it is the first configuration, wait for the start command.
                while d.start == 0:
                    pass
                if d.start == 1:
                    # Start the record and tracker nodes.
                    dh.record_pub.publish(1)
                    dh.tracker_pub.publish(1)
                    d.start = -1

            net = define_net(config[c], dh.phase)
            if recover == 1 and d.state != -1:
                # Recovering state of the net and setting the running attribute.
                net.state = d.state
                net.running = 1

            # Defining the timers of the time transitions.
            if recover == 1 and d.timers != -1:
                timers = d.timers
                index = net.transitions.index('time_limit_password')
                timers[index] = timers[index] + (time.time() - d.shutdown_time)
            else:
                timers = [0] * net.n_transitions

            dh.screen_pub.publish(-2)  # Blank screen.
            dh.zero_for_phase1()

            # Defining password.
            if recover == 1 and d.password != -1:
                dh.password = d.password
            else:
                dh.password = define_password()
            d.password = dh.password

            # Defining virtual agent.
            dh.virtual_agent = va_order[c]
            d.virtual_agent = dh.virtual_agent

            dh.phase = 0

            if recover == 1:
                if dh.configuration != 0:
                    # Starting the record and tracker nodes.
                    dh.record_pub.publish(1)
                    dh.tracker_pub.publish(1)

                # Recovering state of the screen application.
                dh.screen_pub.publish(10)

                # Sending the password to the screen application.
                msg = Int16MultiArray()
                msg.data = dh.password
                dh.pass_pub.publish(msg)

                # Configuring the virtual agent.
                dh.phase_pub.publish(1)
                dh.va_pub.publish(dh.virtual_agent)

                time.sleep(3)

                if net.state[net.places.index('kinect_detection')] > 0 or \
                        net.state[net.places.index('kinect_detection1')] > 0:
                    # If the system was shutdown in the kinect_detection step,
                    # update detection_fail attribute so tracker does not
                    # restart again.
                    dh.detection_fail = -1

                # Updating number of errors.
                dh.count_errors = d.password_errors
                dh.total_errors = dh.count_errors

                # Updating time.
                dh.total_time[0] = d.interaction_time
                if d.start_time != 0:
                    dh.start_time = time.time()

                recover = 0

            # Defining phase's time limits.
            # Time limits for both configurations are 4 minutes.
            net.times[net.transitions.index('time_limit_password')] = 240

            # Writing settings to the log file.
            dh.write_log(['CONFIG'], config_names[config[c]])
            dh.write_log(['VA'], dh.virtual_agent)
            text = str(dh.password)
            dh.write_log(['PASS'], text[1:len(text) - 1])

        if dh.phase == 2:
            # Second phase (counting).
            d.phase = dh.phase
            d.config = c
            d.config_name = config_names[config[c]]

            net = define_net(config[c], dh.phase)

            if recover == 1 and d.state != -1:
                # Recovering state of the net and setting the running attribute.
                net.state = d.state
                net.running = 1

            # Opening counting screen.
            if recover == 0:
                dh.screen_pub.publish(2)

            dh.zero_for_phase2()

            # Setting the counting application.
            dh.counting = counting_values[c]
            dh.counting_images_set = images_set[c]
            d.images_set = images_set[c]

            # Defining the timers of the time transitions.
            if recover == 1 and d.timers != -1:
                timers = d.timers
                index = net.transitions.index('time_limit_counting')
                timers[index] = timers[index] + (time.time() - d.shutdown_time)

                index = net.transitions.index('repeat_instructions')
                timers[index] = timers[index] + (time.time() - d.shutdown_time)
            else:
                timers = [0] * net.n_transitions

            dh.phase = 0

            if recover == 1:
                # Starting the record and tracker nodes.
                dh.record_pub.publish(1)
                dh.tracker_pub.publish(1)

                # Defining and configuring virtual agent.
                dh.virtual_agent = va_order[c]
                d.virtual_agent = dh.virtual_agent
                if net.state[net.places.index('waiting_start')] > 0:
                    dh.phase_pub.publish(1)
                    dh.va_pub.publish(dh.virtual_agent)
                else:
                    dh.phase_pub.publish(2)
                    dh.va_pub.publish(dh.virtual_agent)
                    d.sc_pub.publish(-1)

                time.sleep(2)

                if net.state[net.places.index('waiting_start')] > 0:
                    dh.phase_pub.publish(2)

                # Recovering state of the screen application.
                dh.screen_pub.publish(10)

                time.sleep(1)

                if config_names[config[c]] == 'EXIM/EXIM':
                    # The human gaze system needs to be restarted only in EXIM/EXIM
                    # configuration.
                    if net.state[net.places.index('reference_detection')] > 0:
                        print("human_gaze")
                        # If the system was shutdown in the reference_detection step,
                        # send command to start the human gaze system again.
                        dh.human_gaze_pub.publish(1)

                # Updating added entries.
                dh.added_entries[1] = d.entries

                # Updating time.
                dh.total_time[0] = d.interaction_time
                if d.start_time != 0:
                    dh.start_time = time.time()

                recover = 0

            # Defining phase's time limits.
            # Time limits for both configurations are 4 minutes.
            net.times[net.transitions.index('time_limit_counting')] = 240

            # Writing settings to the log file.
            name = d.pkg_path + "/log/" + log_name
            file = open(name, "a")
            file.write("\n")
            file.close()
            dh.write_log(['IMAGES'], dh.counting_images_set)
            text = str(dh.counting)
            dh.write_log(['VALUES'], text[1:len(text) - 1])

        if net:
            # If a net was already selected.

            # Checking the enabled transitions.
            enabled = net.check_transitions()

            # Choosing the transition to fire.
            if config[c] == 0:
                # EX/EX configuration.
                transition_to_fire = ex_ex_config.choose_transition(net,
                                                                    enabled,
                                                                    timers,
                                                                    dh)
            if config[c] == 1:
                # EXIM/EXIM configuration
                transition_to_fire = exim_exim_config.choose_transition(net,
                                                                        enabled,
                                                                        timers,
                                                                        dh)

            if transition_to_fire != -1:
                # If there is a transition to be fired.

                # Setting the firing vector and updating the state of the net.
                u = np.zeros((1, net.n_transitions))
                u[0][transition_to_fire] = 1
                net.state_transition(u)

                d.state = net.state.tolist()
                d.timers = timers
                d.password_status = dh.password_status
                d.password_errors = dh.count_errors
                d.entries = dh.added_entries[1]
                d.interaction_time = dh.total_time[0]
                if dh.start_time != 0:
                    d.interaction_time = dh.total_time[0] + (time.time() - dh.start_time)
                d.start_time = dh.start_time

                print('Transition fired: ' + net.transitions[transition_to_fire])
                print('')

                # Writing fired transition to the log file.
                if dh.log_file:
                    dh.write_log(['TRANS'], net.transitions[transition_to_fire])

                if 'end_counting_phase' in net.places:
                    # If it is in phase 2.
                    index = net.places.index('end_counting_phase')
                    if net.state[index] == 1:
                        # If the phase finished.

                        # Writing to the log file.
                        dh.log_file.disable(dh.log_file.NOTSET)
                        dh.log_file.info('End.\n')
                        dh.log_file.disable(dh.log_file.WARNING)
                        name = d.pkg_path + "/log/" + log_name
                        file = open(name, "a")
                        file.close()

                        time.sleep(1)

                        # Opening message screen.
                        dh.screen_pub.publish(c+3)

                        # Waiting for a command to continue.
                        while dh.questionnaire != 1:
                            pass

                        # Updating the configuration.
                        c = c + 1
                        if c == len(config) - 1:
                            # If it is the last configuration.
                            dh.configuration = 2
                        else:
                            # If it is not the last configuration.
                            dh.configuration = 1

                        if c == len(config):
                            # If all the configurations were executed already,
                            # stops the record node and inform the end.
                            dh.record_pub.publish(0)
                            end_pub.publish(1)
                            print("End of all configurations")
                            break

            rate.sleep()


if __name__ == '__main__':
    main()