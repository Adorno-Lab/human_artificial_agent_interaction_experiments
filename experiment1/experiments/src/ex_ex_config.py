#!/usr/bin/env python3.6
import roslib
import rospy
import time
import csv
import rospkg
import numpy as np
from std_msgs.msg import Int16, String, Float32, Int16MultiArray
from geometry_msgs.msg import Point
from include.ExPhaseOne import *
from include.ExPhaseTwo import *
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


def voice_and_wait(dh, voice_queue):
    """
    Sends voice commands and waits until they finish.

    :param dh: a DataHandler object.
    :param voice_queue: a list with the voice commands to be sent.
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
        dh.va_speaking = 2

    # Returning the attribute to the correct value.
    if dh.va_speaking == 2:
        dh.va_speaking = 1

    # Waiting until the last audio finishes.
    while dh.va_speaking == 1:
        pass

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

            dh.screen_pub.publish(-1)  # Close surrender pose message.
            if dh.configuration == 0:
                # If it is the first configuration.
                voice_and_wait(dh, ['deteccao_ok', 'comecar'])
                dh.screen_pub.publish(1)  # Password screen.
                voice_and_wait(dh, ['instrucoes_senha'])
                voice_and_wait(dh, ['preste_atencao0'])
            else:
                voice_and_wait(dh, ['deteccao_ok'])
                dh.screen_pub.publish(1)  # Password screen.
                voice_and_wait(dh, ['preste_atencao1'])

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

    if 'detected' in enabled_names:
        # If already detected, publish the command to show password.
        transition = net.transitions.index('detected')
        if dh.detecting_flag == 1:
            dh.screen_pub.publish(1)  # Password screen.

            if dh.configuration == 0:
                # If it is the first configuration.
                voice_and_wait(dh, ['instrucoes_senha'])
                voice_and_wait(dh, ['preste_atencao0'])
            else:
                voice_and_wait(dh, ['preste_atencao1'])

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

            timers[net.transitions.index('problem_start')] = time.time()
            return transition

    if 'kinect_start1' in enabled_names:
        # Start detection again.
        transition = net.transitions.index('kinect_start1')
        if dh.detecting_flag == 0:
            dh.password_pub.publish(-1)  # Disable screen application.

            dh.tracker_pub.publish(1)  # Restart the tracker application.

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
            dh.screen_pub.publish(-1)  # Close surrender pose message.
            voice_and_wait(dh, ['continuar'])

            timers[net.transitions.index('kinect_time1')] = 0
            timers[net.transitions.index('problem_start')] = time.time()

            # Enable screen application again.
            dh.password_pub.publish(0)

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

            dh.check_indicated = 0
            timers[net.transitions.index('correct_wait')] = time.time()
            return transition

    if 'gesture_wrong' in enabled_names:
        # The last indication was wrong.
        transition = net.transitions.index('gesture_wrong')
        if dh.check_indicated == -1:
            dh.password_pub.publish(-1)  # Disable screen application.

            time.sleep(1)

            # Update counting of errors.
            dh.count_errors = dh.count_errors + 1
            dh.total_errors = dh.total_errors + 1

            dh.check_indicated = 0
            timers[net.transitions.index('wrong_not_limit')] = time.time()
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

            voice_and_wait(dh, ['senha_ok'])

            if dh.configuration == 0:
                # If it is the first configuration.
                voice_and_wait(dh, ['posicao_contagem0'])
            else:
                voice_and_wait(dh, ['posicao_contagem1'])

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

                dh.pointed_object = 0
                timers[transition] = 0
                timers[net.transitions.index('problem_start')] = time.time()
                return transition

    if 'stop_pointing' in enabled_names:
        # Human stopped pointing.
        transition = net.transitions.index('stop_pointing')
        if dh.password_status != -1:
            if dh.pointing_flag == 0:
                # If the person stopped pointing after a correct indication,

                dh.password_pub.publish(0)  # Enable screen application.

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

            voice_and_wait(dh, ['repetir_senha'])

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
        if time.time() - timers[transition] >= net.times[transition]:

            # If virtual agent's implicit communication was being used, it would
            # be raising its eyebrows.
            time.sleep(0.5)

            dh.pointed_object = 0
            timers[transition] = 0
            timers[net.transitions.index('problem_start')] = time.time()

            dh.password_pub.publish(0)  # Enable screen application.

            return transition

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
            voice_and_wait(dh, ['ajuda'])
            voice_and_wait(dh, ['repetir_senha'])

            # Waiting for the person to look to the screen.
            time.sleep(1)

            # Show the password and wait until it finishes.
            t_password = 1
            dh.password_pub.publish(t_password)
            time.sleep(4 * t_password)

            # If virtual agent's implicit communication was being used, it would
            # be raising its eyebrows and looking to the person before looking
            # to the correct object.
            time.sleep(0.5)
            time.sleep(2)

            return transition

    if 'problem_pointing' in enabled_names:
        # If human is pointing, inform that does not understand.
        transition = net.transitions.index('problem_pointing')
        if dh.detecting_flag == 1 and dh.pointing_flag > 0:
            voice_and_wait(dh, ['nao_entendo'])
            return transition

    if 'problem_end' in enabled_names:
        # Finish the problem check and restart the timer.
        transition = net.transitions.index('problem_end')
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

                voice_and_wait(dh, ['preencher_senha'])

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

            voice_and_wait(dh, ['senha_ok'])
            if dh.configuration == 0:
                # If it is the first configuration.
                voice_and_wait(dh, ['posicao_contagem0'])
            else:
                voice_and_wait(dh, ['posicao_contagem1'])

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
            dh.password_pub.publish(-1)  # Disable screen application.

            # Set the virtual agent and the password.
            dh.va_pub.publish(dh.virtual_agent)
            msg = Int16MultiArray()
            msg.data = dh.password
            dh.pass_pub.publish(msg)

            # Wait the virtual agent to load.
            time.sleep(3)

            # Virtual agent introduces itself.
            voice_and_wait(dh, ['apresentacao'])
            voice_and_wait(dh, ['quarto'])

            return transition

    # ------------------------ Phase 2 ------------------------ #

    if 'start_command' in enabled_names:
        # If there is a command, give instructions.
        transition = net.transitions.index('start_command')
        if dh.counting_status == -1:

            # Wait the virtual agent to load.
            time.sleep(3)

            if dh.configuration == 0:
                # If it is the first configuration.
                voice_and_wait(dh, ['instrucoes_contagem'])
                voice_and_wait(dh, ['checar_identidade0'])
            else:
                voice_and_wait(dh, ['checar_identidade1'])

            # Waiting for the person to look to the camera.
            time.sleep(2)

            timers[net.transitions.index('enable_fields')] = time.time()
            return transition

    if 'enable_fields' in enabled_names:
        # Enable the fields in the counting application after a time waiting.
        transition = net.transitions.index('enable_fields')
        if time.time() - timers[transition] >= net.times[transition]:
            voice_and_wait(dh, ['olhar_ok'])

            # Enable the screen application.
            dh.counting_pub.publish(0)

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

            # Zero the voice commands.
            dh.voice_pub.publish('0')

            dh.phase_pub.publish(2)
            net.running = 1

            # Set the counting values.
            msg = Int16MultiArray()
            msg.data = [dh.counting_images_set] + dh.counting
            dh.counting_settings_pub.publish(msg)

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

            if dh.log_file:
                dh.write_log(['SCREEN', 'CLOSE'], dh.added_entries[0])

                # Waiting to receive the added value.
                while dh.entry_value == -1:
                    pass
                dh.write_log(['ENTRY'], dh.entry_value)

            timers[net.transitions.index('repeat_instructions')] = 0
            dh.entry_value = -1
            return transition

    if 'repeat_instructions' in enabled_names:
        # Repeat instructions after a time without screen commands.
        transition = net.transitions.index('repeat_instructions')
        if timers[transition] != 0:
            if time.time() - timers[transition] >= net.times[transition]:
                voice_and_wait(dh, ['muito_dificil'])
                voice_and_wait(dh, ['repetir_instrucoes'])

                timers[transition] = 0
                return transition

    if 'time_limit_counting' in enabled_names:
        # If phase's time limit is reached, finish the counting.
        transition = net.transitions.index('time_limit_counting')
        if time.time() - timers[transition] >= net.times[transition]:
            timers[transition] = 0

            voice_and_wait(dh, ['preencher_valores'])

            # Send command to finish the counting.
            dh.counting_pub.publish(-2)
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

            voice_and_wait(dh, ['contagem_ok'])

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

            voice_and_wait(dh, ['contagem_ok'])

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

    return -1


def main():
    net = ExPhaseOne()
    timers = [0]*net.n_transitions  # Timers for transitions.

    rospy.init_node('ex_ex_node')

    dh = DataHandler()
    dh.phase = 1
    dh.password = [1, 2, 3, 4]

    dh.tracker_pub.publish(1)

    rate = rospy.Rate(50)

    count = 0
    va_order = ['luna', 'sofia']

    while not rospy.is_shutdown():
        if dh.phase == 1:
            net = ExPhaseOne()
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
            net = ExPhaseTwo()
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
