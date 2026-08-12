#!/usr/bin/env python3.6
import roslib
import rospy
import time
import csv
import rospkg
import numpy as np
from std_msgs.msg import Int16, String, Float32, Int16MultiArray
from geometry_msgs.msg import Point
from include.ExImPhaseOne import *
from include.ExImPhaseTwo import *
from include.DataHandler import *


def update_speaking_time(dh, voice_queue):
    """
    Updates the total interaction time considering the shorter audio option.
    The audio files have different durations for each virtual agents.

    :param dh: a DataHandler object.
    :param voice_queue: a list with voice audios executed.
    :return:
    """
    # Getting the path for the virtual agent's ROS package.
    rospack = rospkg.RosPack()
    pkg_path = rospack.get_path('virtual-agent')

    # Getting the durations from a file and updating the interaction time.
    with open(pkg_path + "/src/audio-files/audios.csv") as file:
        for line in csv.reader(file):
            if line[0] in voice_queue:
                times = [float(line[1]), float(line[2])]

                # Subtracting the duration of the executed audio file.
                if dh.virtual_agent == 'luna':
                    dh.total_time[0] = dh.total_time[0] - times[0]
                if dh.virtual_agent == 'sofia':
                    dh.total_time[0] = dh.total_time[0] - times[1]

                # Adding the duration of the shorter audio file.
                dh.total_time[0] = dh.total_time[0] + min(times)


def voice_and_wait(dh, voice_queue, gaze_to_human=0):
    """
    Sends voice commands and waits until they finish.

    :param dh: a DataHandler object.
    :param voice_queue: a list with the voice commands to be sent.
    :param gaze_to_human: to update the VA's gaze to the human while speaking.
    :return:
    """
    # Disable clicks in the counting application while the virtual agent is speaking.
    dh.control_clicks_pub.publish(0)

    time.sleep(1)
    for i in range(0, len(voice_queue)):
        dh.voice_pub.publish(voice_queue[i])

        # Waiting until the last command starts to play to send a new one to
        # guarantee that will be received.
        while dh.va_speaking != 1:
            if dh.va_speaking == -1:
                # Error finding the audio file.
                if dh.log_file:
                    msg = 'Unknown voice command: ' + voice_queue[i]
                    dh.write_log(['VOICE', 'ERROR'], msg)
                if i == len(voice_queue) - 1:
                    return
                break
            if gaze_to_human == 1:
                # If the VA's should update the gaze to the human while waiting.
                if dh.x0_head != DQ([1]) and dh.detecting_flag == 1:
                    dh.va_gaze_pub.publish(dh.set_odometry_msg(dh.x0_head))

        dh.va_speaking = 2

    # Returning the attribute to the correct value.
    if dh.va_speaking == 2:
        dh.va_speaking = 1

    # Waiting until the last audio finishes.
    while dh.va_speaking == 1:
        if gaze_to_human == 1:
            # If the VA's should update the gaze to the human while waiting.
            if dh.x0_head != DQ([1]) and dh.detecting_flag == 1:
                dh.va_gaze_pub.publish(dh.set_odometry_msg(dh.x0_head))

    # Updating interaction time.
    if dh.start_time != 0:
        dh.total_time[0] = dh.total_time[0] + (time.time() - dh.start_time)
        update_speaking_time(dh, voice_queue)
        dh.start_time = time.time()

    # Enable clicks in the counting application again.
    dh.control_clicks_pub.publish(1)


def choose_transition(net, enabled, timers, dh):
    # Transitions are checked in order of priority to fire.

    # Get the enabled transitions names.
    enabled_names = []
    for i in enabled:
        enabled_names.append(net.transitions[i])

    # ------------------------ Phase 1 ------------------------ #

    if 'kinect_start' in enabled_names:
        # Ask for surrender pose and start the detection timer.
        transition = net.transitions.index('kinect_start')
        if dh.detecting_flag == 0:
            dh.x0_head = DQ([1])
            dh.expression_pub.publish('neutral')

            if dh.configuration == 0:
                # If it is the first configuration.
                voice_and_wait(dh, ['encontrar_voce', 'pose_deteccao0'])
            else:
                voice_and_wait(dh, ['pose_deteccao1'])
            dh.screen_pub.publish(-1)  # Surrender pose message.
            timers[net.transitions.index('kinect_time')] = time.time()

            return transition

    if 'kinect_time' in enabled_names:
        # If the time is up, ask for the surrender pose again.
        transition = net.transitions.index('kinect_time')
        if time.time() - timers[transition] >= net.times[transition]:
            dh.detection_fail = dh.detection_fail + 1
            if dh.detection_fail == dh.max_detection_failures:
                # If the transition fires a few times in a row, stop the tracker.
                dh.tracker_pub.publish(0)

            voice_and_wait(dh, ['abaixe_bracos'])

            if dh.detection_fail == dh.max_detection_failures:
                # Restart the tracker.
                dh.tracker_pub.publish(1)
                dh.detection_fail = 0

            timers[transition] = time.time()
            return transition

    if 'kinect_end' in enabled_names:
        # If detected, zero the detection timer, start the timer for
        # start_accepting and publish the command to show password.
        transition = net.transitions.index('kinect_end')
        if dh.detecting_flag == 1:
            timers[net.transitions.index('kinect_time')] = 0

            # Virtual agent is happy and looks to the human.
            dh.expression_pub.publish('happy')
            if dh.x0_head != DQ([1]) and dh.detecting_flag == 1:
                dh.va_gaze_pub.publish(dh.set_odometry_msg(dh.x0_head))
            dh.last_va_gaze = 1

            dh.screen_pub.publish(-1)  # Close surrender pose message.

            if dh.configuration == 0:
                # If it is the first configuration.
                voice_and_wait(dh, ['deteccao_ok', 'comecar'], 1)
                dh.screen_pub.publish(1)  # Password screen.
                dh.expression_pub.publish('neutral')
                voice_and_wait(dh, ['instrucoes_senha'], 1)
                voice_and_wait(dh, ['preste_atencao0'], 1)
            else:
                voice_and_wait(dh, ['deteccao_ok'], 1)
                dh.screen_pub.publish(1)  # Password screen.
                voice_and_wait(dh, ['preste_atencao1'], 1)

            # Waiting for the person to look to the screen.
            time.sleep(1)

            # Define time for the password and configure start_accepting timer.
            t_password = 1
            net.times[net.transitions.index('start_accepting')] = 4*t_password
            timers[net.transitions.index('start_accepting')] = time.time()
            dh.password_pub.publish(t_password)

            # Start counting the interaction time in phase 1.
            dh.start_time = time.time()

            return transition

    if 'detected' in enabled_names:
        # If already detected, publish the command to show password.
        transition = net.transitions.index('detected')
        if dh.detecting_flag == 1:
            dh.expression_pub.publish('neutral')
            # Virtual agent looks to the human.
            if dh.x0_head != DQ([1]) and dh.detecting_flag == 1:
                dh.va_gaze_pub.publish(dh.set_odometry_msg(dh.x0_head))
            dh.last_va_gaze = 1

            dh.screen_pub.publish(1)  # Password screen.

            if dh.configuration == 0:
                # If it is the first configuration.
                voice_and_wait(dh, ['instrucoes_senha'], 1)
                voice_and_wait(dh, ['preste_atencao0'], 1)
            else:
                voice_and_wait(dh, ['preste_atencao1'], 1)

            # Waiting for the person to look to the screen.
            time.sleep(1)

            # Define time for the password and configure start_accepting timer.
            t_password = 1
            net.times[net.transitions.index('start_accepting')] = 4 * t_password
            timers[net.transitions.index('start_accepting')] = time.time()
            dh.password_pub.publish(t_password)

            # Start counting the interaction time in phase 1.
            dh.start_time = time.time()

            return transition

    if 'start_accepting' in enabled_names:
        # If ready to start accepting gesture commands, publish the command to
        # enable the screen application.
        transition = net.transitions.index('start_accepting')

        if time.time() - timers[transition] >= net.times[transition]:
            timers[transition] = 0
            dh.pointed_object = 0
            dh.password_pub.publish(0)  # Enable screen application.

            # Virtual agent's expression is neutral.
            dh.expression_pub.publish('neutral')

            # Virtual agent looks to the next correct indication.
            next = dh.password[dh.password_status] - 1
            dh.va_gaze_pub.publish(dh.set_odometry_msg(dh.x0_objects[next]))
            dh.last_va_gaze = 2

            timers[net.transitions.index('problem_start')] = time.time()
            return transition

    if 'kinect_start1' in enabled_names:
        # Start detection again.
        transition = net.transitions.index('kinect_start1')
        if dh.detecting_flag == 0:
            dh.x0_head = DQ([1])

            dh.tracker_pub.publish(1)  # Restart the tracker application.

            dh.password_pub.publish(-1)  # Disable screen application.

            dh.expression_pub.publish('neutral')
            voice_and_wait(dh, ['pose_deteccao1'])
            dh.screen_pub.publish(-1)  # Surrender pose message.

            timers[net.transitions.index('kinect_time1')] = time.time()
            return transition

    if 'kinect_time1' in enabled_names:
        # If the time is up, ask for the surrender pose again.
        transition = net.transitions.index('kinect_time1')
        if time.time() - timers[transition] >= net.times[transition]:
            dh.detection_fail = dh.detection_fail + 1
            if dh.detection_fail == dh.max_detection_failures:
                # If the transition fires a few times in a row, stop the tracker.
                dh.tracker_pub.publish(0)

            voice_and_wait(dh, ['abaixe_bracos'])

            if dh.detection_fail == dh.max_detection_failures:
                # Restart the tracker.
                dh.tracker_pub.publish(1)
                dh.detection_fail = 0

            timers[transition] = time.time()
            return transition

    if 'kinect_end1' in enabled_names:
        # If detected, zero the detection timer.
        transition = net.transitions.index('kinect_end1')
        if dh.detecting_flag == 1:

            # Virtual agent is happy and looks to the human.
            dh.expression_pub.publish('happy')
            if dh.x0_head != DQ([1]) and dh.detecting_flag == 1:
                dh.va_gaze_pub.publish(dh.set_odometry_msg(dh.x0_head))

            dh.screen_pub.publish(-1)  # Close surrender pose message.
            voice_and_wait(dh, ['continuar'], 1)

            # Virtual agent's expression is neutral.
            dh.expression_pub.publish('neutral')

            # Enable screen application again.
            dh.password_pub.publish(0)

            # Virtual agent looks to the next correct indication.
            next = dh.password[dh.password_status] - 1
            dh.va_gaze_pub.publish(dh.set_odometry_msg(dh.x0_objects[next]))
            dh.last_va_gaze = 2

            timers[net.transitions.index('kinect_time1')] = 0
            timers[net.transitions.index('problem_start')] = time.time()

            # Updating start_time.
            dh.start_time = time.time()
            return transition

    if 'gesture_start' in enabled_names:
        # The indication of an object was detected.
        transition = net.transitions.index('gesture_start')

        if dh.detecting_flag == 1 and dh.pointed_object > 0:
            timers[net.transitions.index('problem_start')] = 0
            dh.password_pub.publish(0)  # Enable screen application.

            if dh.log_file:
                dh.write_log(['GESTURE'], dh.pointed_object)

            # Publish the indicated color.
            if dh.pointed_object == 1:
                dh.indication_pub.publish('blue')
            if dh.pointed_object == 2:
                dh.indication_pub.publish('yellow')
            if dh.pointed_object == 3:
                dh.indication_pub.publish('white')
            if dh.pointed_object == 4:
                dh.indication_pub.publish('black')

            dh.pointed_object = 0
            return transition

    if 'gesture_correct' in enabled_names:
        # The last indication was correct.
        transition = net.transitions.index('gesture_correct')
        if dh.check_indicated == 1:
            dh.password_pub.publish(-1)  # Disable screen application.

            dh.count_correct = dh.count_correct + 1
            if dh.count_correct < len(dh.password):
                # Virtual agent is satisfied while still looking to the object.
                dh.expression_pub.publish('satisfied')
                dh.last_va_gaze = 0
            else:
                # Virtual agent is happy and looks to the human.
                dh.expression_pub.publish('happy')
                dh.va_gaze_pub.publish(dh.set_odometry_msg(dh.x0_head))
                dh.last_va_gaze = 1

            dh.check_indicated = 0
            timers[net.transitions.index('correct_wait')] = time.time()
            return transition

    if 'gesture_wrong' in enabled_names:
        # The last indication was wrong.
        transition = net.transitions.index('gesture_wrong')
        if dh.check_indicated == -1:
            dh.password_pub.publish(-1)  # Disable screen application.

            # Virtual agent is sad.
            dh.expression_pub.publish('sad')
            time.sleep(1)

            # Update counting of errors.
            dh.count_errors = dh.count_errors + 1
            dh.total_errors = dh.total_errors + 1

            dh.check_indicated = 0
            return transition

    if 'password_finished' in enabled_names:
        # Password finished.
        transition = net.transitions.index('password_finished')
        if dh.password_status == -1:

            # Saving the total time of phase 1.
            dh.total_time[0] = dh.total_time[0] + (time.time() - dh.start_time)
            dh.total_time[1] = dh.total_time[0]
            dh.total_time[0] = 0
            dh.start_time = 0

            voice_and_wait(dh, ['senha_ok'], 1)

            # Virtual agent's expression is neutral.
            dh.expression_pub.publish('neutral')

            if dh.configuration == 0:
                # If it is the first configuration.
                voice_and_wait(dh, ['posicao_contagem0'], 1)
            else:
                voice_and_wait(dh, ['posicao_contagem1'], 1)

            net.running = 0
            dh.phase = 2

            if dh.log_file:
                dh.write_log(['PHASE1', 'ERRORS'], dh.total_errors)
                dh.write_log(['PHASE1', 'TIME'], dh.total_time[1])

            return transition

    if 'correct_wait' in enabled_names:
        # Waiting after correct.
        transition = net.transitions.index('correct_wait')
        if dh.password_status != -1:
            if time.time() - timers[transition] >= net.times[transition]:
                # If some time has passed after a correct indication.

                dh.password_pub.publish(0)  # Enable screen application.

                # Virtual agent's expression is neutral.
                dh.expression_pub.publish('neutral')

                # Virtual agent looks to the next correct indication.
                next = dh.password[dh.password_status] - 1
                dh.va_gaze_pub.publish(dh.set_odometry_msg(dh.x0_objects[next]))
                dh.last_va_gaze = 2

                dh.pointed_object = 0
                timers[transition] = 0
                timers[net.transitions.index('problem_start')] = time.time()
                return transition

    if 'stop_pointing' in enabled_names:
        # Human stopped pointing.
        transition = net.transitions.index('stop_pointing')
        if dh.password_status != -1:
            if dh.pointing_flag == 0:
                # If the person stopped pointing after a correct indication.

                dh.password_pub.publish(0)  # Enable screen application.

                # Virtual agent's expression is neutral.
                dh.expression_pub.publish('neutral')

                # Virtual agent looks to the next correct indication.
                next = dh.password[dh.password_status] - 1
                dh.va_gaze_pub.publish(dh.set_odometry_msg(dh.x0_objects[next]))
                dh.last_va_gaze = 2

                dh.pointed_object = 0
                timers[net.transitions.index('correct_wait')] = 0
                timers[net.transitions.index('problem_start')] = time.time()
                return transition

    if 'error_limit' in enabled_names:
        # Limit of errors reached.
        transition = net.transitions.index('error_limit')
        if dh.count_errors >= dh.max_errors:
            dh.count_errors = 0
            dh.pointed_object = 0

            # Virtual agent's expression is neutral and it looks to the human.
            dh.expression_pub.publish('neutral')
            dh.va_gaze_pub.publish(dh.set_odometry_msg(dh.x0_head))
            dh.last_va_gaze = 1

            voice_and_wait(dh, ['repetir_senha'], 1)

            # Waiting for the person to look to the screen.
            time.sleep(1)

            # Define time for the password and configure start_accepting timer.
            t_password = 1
            net.times[net.transitions.index('start_accepting')] = 4 * t_password
            timers[net.transitions.index('start_accepting')] = time.time()
            dh.password_pub.publish(t_password)
            return transition

    if 'wrong_not_limit' in enabled_names:
        # Wrong indication but error limit not reached.
        transition = net.transitions.index('wrong_not_limit')

        # Virtual agent looks to the human.
        dh.va_gaze_pub.publish(dh.set_odometry_msg(dh.x0_head))
        dh.last_va_gaze = 1

        # Virtual agent raises its eyebrows.
        dh.expression_pub.publish('neutral')
        time.sleep(0.5)
        dh.expression_pub.publish('eyebrows')

        # Virtual agent looking to the person before looking to the correct
        # object.
        start = time.time()
        while time.time() - start < 2:
            dh.va_gaze_pub.publish(dh.set_odometry_msg(dh.x0_head))

        # Virtual agent's expression is neutral.
        dh.expression_pub.publish('neutral')

        # Virtual agent looks to the next correct indication.
        next = dh.password[dh.password_status] - 1
        dh.va_gaze_pub.publish(dh.set_odometry_msg(dh.x0_objects[next]))
        dh.last_va_gaze = 2

        dh.pointed_object = 0
        timers[transition] = 0
        timers[net.transitions.index('problem_start')] = time.time()

        dh.password_pub.publish(0)  # Enable screen application.
        return transition

    """if 'information_area' in enabled_names:
        # If human is in the help area.
        transition = net.transitions.index('information_area')
        if dh.id_help not in dh.markers_ids:
            # Virtual agent looks to the human.
            dh.va_gaze_pub.publish(dh.set_odometry_msg(dh.x0_head))
            dh.last_va_gaze = 1

            voice_and_wait(dh, ['ajuda'], 1)
            return transition

    if 'finish_help' in enabled_names:
        # If virtual agent finished the instructions to help the human.
        transition = net.transitions.index('finish_help')

        # Waiting for the person to look to the screen.
        time.sleep(1)

        # Define time for the password and configure start_accepting timer.
        t_password = 1
        net.times[net.transitions.index('start_accepting')] = 4 * t_password
        timers[net.transitions.index('start_accepting')] = time.time()
        dh.password_pub.publish(t_password)
        return transition"""

    if 'problem_start' in enabled_names:
        # If time is up, start the procedure for problem detection.
        transition = net.transitions.index('problem_start')
        if time.time() - timers[transition] >= net.times[transition]:
            timers[transition] = 0
            return transition

    if 'problem_not_tracking' in enabled_names:
        # If human is not being tracked, inform the problem.
        transition = net.transitions.index('problem_not_tracking')
        if dh.detecting_flag == 0:
            dh.tracker_pub.publish(0)  # Disable tracker application.

            # Virtual agent is sad and looks to the reference.
            dh.expression_pub.publish('sad')
            point = 1 + DQ.E * 0.5 * DQ([0, 0, 0]) * 1
            dh.va_gaze_pub.publish(dh.set_odometry_msg(point))
            dh.last_va_gaze = 0
            dh.x0_head = DQ([1])

            voice_and_wait(dh, ['perdi_voce'])

            # Updating total time.
            # It will not consider the time to detect the human again.
            if dh.start_time != 0:
                dh.total_time[0] = dh.total_time[0] + (time.time() - dh.start_time)
            dh.start_time = 0
            return transition

    if 'problem_tracking' in enabled_names:
        # If human is being tracked, continue to look for the problem.
        transition = net.transitions.index('problem_tracking')
        if dh.detecting_flag == 1:
            return transition

    if 'problem_not_pointing' in enabled_names:
        # If human is not pointing, virtual agent helps.
        transition = net.transitions.index('problem_not_pointing')
        if dh.pointing_flag == 0:

            # Virtual agent is thinking and looks to the human.
            dh.expression_pub.publish('thinking')
            dh.va_gaze_pub.publish(dh.set_odometry_msg(dh.x0_head))

            voice_and_wait(dh, ['ajuda'], 1)

            # Virtual agent's expression is neutral.
            dh.expression_pub.publish('neutral')

            voice_and_wait(dh, ['repetir_senha'])

            # Waiting for the person to look to the screen.
            time.sleep(1)

            # Show the password and wait until it finishes.
            t_password = 1
            dh.password_pub.publish(t_password)
            time.sleep(4 * t_password)

            # Virtual agent raises its eyebrows.
            time.sleep(0.5)
            dh.expression_pub.publish('eyebrows')

            # Virtual agent looking to the person before looking to the correct
            # object.
            start = time.time()
            while time.time() - start < 2:
                dh.va_gaze_pub.publish(dh.set_odometry_msg(dh.x0_head))
            dh.last_va_gaze = 1

            # Virtual agent's expression is neutral.
            dh.expression_pub.publish('neutral')

            # Virtual agent looks to the next correct indication.
            next = dh.password[dh.password_status] - 1
            dh.va_gaze_pub.publish(dh.set_odometry_msg(dh.x0_objects[next]))
            dh.last_va_gaze = 2

            return transition

    if 'problem_pointing' in enabled_names:
        # If human is pointing, inform that does not understand.
        transition = net.transitions.index('problem_pointing')
        if dh.detecting_flag == 1 and dh.pointing_flag > 0:
            # Virtual agent is thinking and looks to the human.
            dh.expression_pub.publish('thinking')
            dh.va_gaze_pub.publish(dh.set_odometry_msg(dh.x0_head))
            dh.last_va_gaze = 1

            voice_and_wait(dh, ['nao_entendo'], 1)
            return transition

    if 'problem_end' in enabled_names:
        # Finish the problem check and restart the timer.
        transition = net.transitions.index('problem_end')

        # Virtual agent's expression is neutral.
        dh.expression_pub.publish('neutral')

        timers[net.transitions.index('problem_start')] = time.time()
        return transition

    if 'time_limit_password' in enabled_names:
        # If phase's time limit is reached, finish the password.
        transition = net.transitions.index('time_limit_password')
        if time.time() - timers[transition] >= net.times[transition]:
            if dh.detecting_flag == 1:
                # Password phase can only finish if human is being tracked.
                timers[transition] = 0
                timers[net.transitions.index('problem_start')] = 0

                # Virtual agent is happy and looks to the human.
                dh.expression_pub.publish('happy')
                dh.va_gaze_pub.publish(dh.set_odometry_msg(dh.x0_head))
                dh.last_va_gaze = 1

                voice_and_wait(dh, ['preencher_senha'], 1)

                if dh.log_file:
                    dh.write_log(['PHASE1', 'ADDED'], dh.password_status)

                # Send command to finish the password.
                dh.password_pub.publish(-2)
                return transition

    if 'password_completed' in enabled_names:
        # Virtual agent completed the password.
        transition = net.transitions.index('password_completed')
        if dh.password_status == -1:

            # Saving the total time of phase 1.
            dh.total_time[0] = dh.total_time[0] + (time.time() - dh.start_time)
            dh.total_time[1] = dh.total_time[0]
            dh.total_time[0] = 0
            dh.start_time = 0

            # Virtual agent's expression is neutral and it looks to the human.
            dh.expression_pub.publish('neutral')
            dh.va_gaze_pub.publish(dh.set_odometry_msg(dh.x0_head))
            dh.last_va_gaze = 1

            voice_and_wait(dh, ['senha_ok'])
            if dh.configuration == 0:
                # If it is the first configuration.
                voice_and_wait(dh, ['posicao_contagem0'], 1)
            else:
                voice_and_wait(dh, ['posicao_contagem1'], 1)

            net.running = 0
            dh.phase = 2

            if dh.log_file:
                dh.write_log(['PHASE1', 'ERRORS'], dh.total_errors)
                dh.write_log(['PHASE1', 'TIME'], dh.total_time[1])

            return transition

    if 'start_password_phase' in enabled_names:
        # If the net is not running, start.
        transition = net.transitions.index('start_password_phase')
        if net.running == 0:

            # Starts phase timer.
            timers[net.transitions.index('time_limit_password')] = time.time()

            # Zero the voice commands.
            dh.voice_pub.publish('0')

            dh.phase_pub.publish(1)
            net.running = 1
            dh.password_pub.publish(-1)

            # Set the virtual agent and the password.
            dh.va_pub.publish(dh.virtual_agent)
            msg = Int16MultiArray()
            msg.data = dh.password
            dh.pass_pub.publish(msg)

            # Virtual agent looks to the reference.
            point = 1 + DQ.E * 0.5 * DQ([0, 0, 0]) * 1
            dh.va_gaze_pub.publish(dh.set_odometry_msg(point))
            dh.last_va_gaze = 0

            # Wait the virtual agent to load.
            time.sleep(3)

            # Virtual agent introduces itself.
            voice_and_wait(dh, ['apresentacao'])

            # Virtual agent is happy and looks to the human.
            dh.expression_pub.publish('happy')
            if dh.x0_head != DQ([1]) and dh.detecting_flag == 1:
                dh.va_gaze_pub.publish(dh.set_odometry_msg(dh.x0_head))
            dh.last_va_gaze = 1

            voice_and_wait(dh, ['quarto'])

            return transition

    # ------------------------ Phase 2 ------------------------ #

    if 'start_command' in enabled_names:
        # If there is a command, give instructions.
        transition = net.transitions.index('start_command')
        if dh.counting_status == -1:

            # Wait the virtual agent to load.
            time.sleep(3)

            # Virtual agent looks to the human.
            dh.va_gaze_pub.publish(dh.set_odometry_msg(dh.x0_head))
            dh.last_va_gaze = 1

            if dh.configuration == 0:
                # If it is the first configuration.
                voice_and_wait(dh, ['instrucoes_contagem'])
                voice_and_wait(dh, ['checar_identidade0'], 1)
            else:
                voice_and_wait(dh, ['checar_identidade1'], 1)

            # Waiting for the person to look to the camera.
            time.sleep(2)

            return transition

    if 'human_positioned' in enabled_names:
        # If the human is in position, give instructions.
        transition = net.transitions.index('human_positioned')

        # If the use of the counting phase marker was disabled but the marker is seen again, enable the communication.
        if dh.counting_marker == 0 and dh.id_counting in dh.markers_ids:
            dh.counting_marker = 1

        if dh.counting_marker == 1 and dh.id_counting not in dh.markers_ids:

            # Send command to start the application with fields disabled.
            dh.counting_pub.publish(-1)

            # Wait the virtual agent to load.
            time.sleep(3)

            # Virtual agent looks to the human.
            dh.va_gaze_pub.publish(dh.set_odometry_msg(dh.x0_head))
            dh.last_va_gaze = 1

            if dh.configuration == 0:
                # If it is the first configuration.
                voice_and_wait(dh, ['instrucoes_contagem'], 1)
                voice_and_wait(dh, ['checar_identidade0'], 1)
            else:
                voice_and_wait(dh, ['checar_identidade1'], 1)

            # Waiting for the person to look to the camera.
            time.sleep(2)
            return transition

    if 'get_reference' in enabled_names:
        # Start the detection of the reference facial points.
        transition = net.transitions.index('get_reference')

        # Send command to start detection.
        dh.human_gaze_pub.publish(1)
        return transition

    if 'enable_fields' in enabled_names:
        # Enable the fields in the counting application.
        transition = net.transitions.index('enable_fields')
        if dh.facial_points == 1:

            # Virtual agent is happy and looks to the human.
            dh.expression_pub.publish('happy')
            dh.va_gaze_pub.publish(dh.set_odometry_msg(dh.x0_head))
            dh.last_va_gaze = 1

            voice_and_wait(dh, ['olhar_ok'], 1)

            # Enable the screen application.
            dh.counting_pub.publish(0)

            # Virtual agent's expression is neutral.
            dh.expression_pub.publish('neutral')

            # Start counting the interaction time in phase 2.
            dh.start_time = time.time()

            timers[net.transitions.index('repeat_instructions')] = time.time()
            return transition

    if 'start_counting_phase' in enabled_names:
        # If the net is not running, start.
        transition = net.transitions.index('start_counting_phase')
        if net.running == 0:

            # Starts phase timer.
            timers[net.transitions.index('time_limit_counting')] = time.time()

            # Zero the voice commands and the expression.
            dh.voice_pub.publish('0')
            dh.expression_pub.publish('neutral')

            dh.phase_pub.publish(2)
            net.running = 1

            # Set the counting values.
            msg = Int16MultiArray()
            msg.data = [dh.counting_images_set] + dh.counting
            dh.counting_settings_pub.publish(msg)

            # Checking if the counting phase marker is being seen. If not, disable the use of it.
            # This is to avoid errors in the process if a problem with the marker detection occurs.
            if dh.id_counting not in dh.markers_ids:
                dh.counting_marker = 0

            return transition

    if 'screen_control_start' in enabled_names:
        # Start control of the main screen place.
        transition = net.transitions.index('screen_control_start')
        return transition

    if 'screen_control_end' in enabled_names:
        # Finish control of the main screen place.
        transition = net.transitions.index('screen_control_end')
        return transition

    if 'screen_start' in enabled_names:
        # A field was selected in the counting application.
        transition = net.transitions.index('screen_start')
        if dh.counting_status > 0:
            dh.added_entries[0] = dh.counting_status

            # Virtual agent looks to the open field.
            point = 1 + DQ.E * 0.5 * (dh.counting_status * DQ([1, 1, 1])) * 1
            dh.va_gaze_pub.publish(dh.set_odometry_msg(point))
            dh.last_va_gaze = 3
            dh.timer_gaze = time.time()

            if dh.log_file:
                dh.write_log(['SCREEN', 'OPEN'], dh.added_entries[0])

            dh.counting_status = -2
            return transition

    if 'screen_end' in enabled_names:
        # An input was given in the counting application.
        transition = net.transitions.index('screen_end')
        if dh.counting_status == 0 or dh.counting_status == -1:

            # Waiting to check the input.
            while dh.check_input == -1:
                pass

            # Update the data about added entries.
            dh.added_entries[1][dh.added_entries[0] - 1] = dh.check_input
            dh.check_input = -1

            # Virtual agent looks to the human.
            dh.va_gaze_pub.publish(dh.set_odometry_msg(dh.x0_head))
            dh.last_va_gaze = 1

            if dh.log_file:
                dh.write_log(['SCREEN', 'CLOSE'], dh.added_entries[0])

                # Waiting to receive the added value.
                while dh.entry_value == -1:
                    pass
                dh.write_log(['ENTRY'], dh.entry_value)

            timers[net.transitions.index('repeat_instructions')] = 0
            dh.looked_area = 0
            dh.entry_value = -1
            return transition

    if 'repeat_instructions' in enabled_names:
        # Repeat instructions after a time without screen commands.
        transition = net.transitions.index('repeat_instructions')
        if timers[transition] != 0:
            if time.time() - timers[transition] >= net.times[transition]:

                # Virtual agent is thinking and looks to the human.
                dh.expression_pub.publish('thinking')
                dh.va_gaze_pub.publish(dh.set_odometry_msg(dh.x0_head))
                dh.last_va_gaze = 1

                voice_and_wait(dh, ['muito_dificil'], 1)

                # Virtual agent's expression is neutral.
                dh.expression_pub.publish('neutral')

                voice_and_wait(dh, ['repetir_instrucoes'], 1)

                timers[transition] = 0
                return transition

    if 'gaze_start' in enabled_names:
        # Human gaze was detected.
        transition = net.transitions.index('gaze_start')
        if dh.counting_status == 0 or time.time() - dh.timer_gaze > 5:
            if dh.looked_area > 0 and dh.added_entries[1][dh.looked_area-1] == -1:
                # If the detected field was not filled yet.

                # Virtual agent looks to the open field.
                point = 1 + DQ.E * 0.5 * (dh.looked_area * DQ([1, 1, 1])) * 1
                dh.va_gaze_pub.publish(dh.set_odometry_msg(point))
                dh.last_va_gaze = 4

                if dh.log_file:
                    dh.write_log(['GAZE'], dh.looked_area)

                if dh.looked_area != dh.last_spoken:
                    # Only speaks if the last time it spoke was not for the
                    # current field.
                    dh.last_spoken = dh.looked_area

                    audio = 'entrada' + str(dh.counting[dh.looked_area - 1])
                    voice_and_wait(dh, [audio])

                    if dh.log_file:
                        dh.write_log(['GAZE', 'HELP'], audio)

                dh.timer_gaze = time.time()
                dh.looked_area = 0
                return transition

    if 'gaze_end' in enabled_names:
        # Finish processing the human gaze.
        transition = net.transitions.index('gaze_end')
        return transition

    if 'time_limit_counting' in enabled_names:
        # If phase's time limit is reached, finish the counting.
        transition = net.transitions.index('time_limit_counting')
        if time.time() - timers[transition] >= net.times[transition]:
            timers[transition] = 0

            # Virtual agent looks to the human.
            dh.va_gaze_pub.publish(dh.set_odometry_msg(dh.x0_head))
            dh.last_va_gaze = 1

            # Virtual agent is happy.
            dh.expression_pub.publish('happy')
            voice_and_wait(dh, ['preencher_valores'], 1)

            # Send command to finish the counting.
            dh.counting_pub.publish(-2)
            dh.timer_gaze = 0
            return transition

    if 'counting_completed' in enabled_names:
        # Phase completed by the virtual agent.
        transition = net.transitions.index('counting_completed')
        if dh.counting_status == -1:

            # Saving the total time of phase 2.
            dh.total_time[0] = dh.total_time[0] + (time.time() - dh.start_time)
            dh.total_time[2] = dh.total_time[0]
            dh.total_time[0] = 0
            dh.start_time = 0

            dh.human_gaze_pub.publish(0)  # Disable gaze detection.

            # Virtual agent's expression is neutral and it looks to the human.
            dh.expression_pub.publish('neutral')
            dh.va_gaze_pub.publish(dh.set_odometry_msg(dh.x0_head))
            dh.last_va_gaze = 1

            voice_and_wait(dh, ['contagem_ok'], 1)

            net.running = 0
            dh.phase = 1

            if dh.log_file:
                added = len(dh.added_entries[1]) - dh.added_entries[1].count(-1)
                dh.write_log(['PHASE2', 'ADDED'], added)
                dh.write_log(['PHASE2', 'ERRORS'], dh.added_entries[1].count(0))
                dh.write_log(['PHASE2', 'TIME'], dh.total_time[2])

            return transition

    if 'counting_finished' in enabled_names:
        # Counting finished.
        transition = net.transitions.index('counting_finished')
        if -1 not in dh.added_entries[1]:

            # Saving the total time of phase 2.
            dh.total_time[0] = dh.total_time[0] + (time.time() - dh.start_time)
            dh.total_time[2] = dh.total_time[0]
            dh.total_time[0] = 0
            dh.start_time = 0

            dh.human_gaze_pub.publish(0)  # Disable gaze detection.

            # Virtual agent is happy and looks to the human.
            dh.expression_pub.publish('happy')
            dh.va_gaze_pub.publish(dh.set_odometry_msg(dh.x0_head))
            dh.last_va_gaze = 1

            voice_and_wait(dh, ['contagem_ok'], 1)

            # Virtual agent's expression is neutral.
            dh.expression_pub.publish('neutral')

            net.running = 0
            dh.phase = 1

            if dh.log_file:
                dh.write_log(['PHASE2', 'ERRORS'], dh.added_entries[1].count(0))
                dh.write_log(['PHASE2', 'TIME'], dh.total_time[2])

            return transition

    if 'counting_not_finished' in enabled_names:
        # Counting not finished.
        transition = net.transitions.index('counting_not_finished')
        if dh.counting_status != -1:
            return transition

    # Updating the virtual agent's gaze when no transition is fired.
    if dh.last_va_gaze == 1:
        # Virtual agent is looking to the human.
        dh.va_gaze_pub.publish(dh.set_odometry_msg(dh.x0_head))

    if dh.last_va_gaze == 2:
        # Virtual agent is looking to the next correct object.
        next = dh.password[dh.password_status] - 1
        dh.va_gaze_pub.publish(dh.set_odometry_msg(dh.x0_objects[next]))

    if dh.last_va_gaze == 3:
        # Virtual agent looks back to the human after looking to the open field.
        if dh.timer_gaze != 0:
            if time.time() - dh.timer_gaze > 5:
                dh.va_gaze_pub.publish(dh.set_odometry_msg(dh.x0_head))
                dh.timer_gaze = 0

    if dh.last_va_gaze == 4:
        # Virtual agent looks back to the human after following her gaze to
        # one of the entry fields.
        if dh.timer_gaze != 0:
            if time.time() - dh.timer_gaze > 3:
                dh.va_gaze_pub.publish(dh.set_odometry_msg(dh.x0_head))
                dh.timer_gaze = 0

    return -1


def main():
    net = ExImPhaseOne()
    timers = [0]*net.n_transitions  # Timers for transitions.

    rospy.init_node('exim_exim_node')

    dh = DataHandler()
    dh.phase = 1
    dh.password = [1, 2, 3, 4]

    dh.tracker_pub.publish(1)

    rate = rospy.Rate(50)

    count = 0
    va_order = ['sofia', 'sofia']

    while not rospy.is_shutdown():
        if dh.phase == 1:
            net = ExImPhaseOne()
            dh.screen_pub.publish(-2)  # Blank screen.
            dh.zero_for_phase1()
            dh.password = [1, 2, 3, 4]
            dh.virtual_agent = va_order[count]
            count = count + 1
            if count == len(va_order):
                count = 0
            timers = [0] * net.n_transitions
            dh.phase = 0
        if dh.phase == 2:
            net = ExImPhaseTwo()
            dh.screen_pub.publish(2)  # Counting screen.
            dh.zero_for_phase2()
            dh.counting = [0, 0, 0, 0]
            dh.counting_images_set = 1
            timers = [0] * net.n_transitions
            dh.phase = 0

        enabled = net.check_transitions()
        transition_to_fire = choose_transition(net, enabled, timers, dh)

        if transition_to_fire != -1:
            u = np.zeros((1, net.n_transitions))
            u[0][transition_to_fire] = 1
            net.state_transition(u)
            print('Transition fired: ' + net.transitions[transition_to_fire])
            print('')
            if dh.log_file:
                dh.write_log(['TRANS'], net.transitions[transition_to_fire])

        rate.sleep()


if __name__ == '__main__':
    main()