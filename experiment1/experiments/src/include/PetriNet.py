import numpy as np
from .Subnet import *


class PetriNet:
    """
    PetriNet is a class for Petri nets.

    Attributes:
        matrix: incidence matrix.
        bi_arcs: matrix indicating the bidirectional arcs.
        transitions: names of transitions.
        places: names of places.
        times: times for the time transitions.
        subnets: included subnets.
        state: current state of the net.
        n_places: number of places.
        n_transitions: number of transitions.
        running: indication if the net is running (1) or not (0).

        Available subnets:
            tracking_subnet: kinect tracking.
            gesture_subnet: gesture command.
            execution_subnet: command execution.
            screen_subnet: screen command.
            problem_check_subnet: problem check.
            finish_subnet: finish net.

    Private methods:
        _create_subnets: creates the Subnet objects.
        _concatenate_subnets: concatenate subnets, initializing the net attributes.
        _add_transition: adds a new transition to the net.
        _add_place: adds a new place to the net.

    Public methods:
        check_transitions: checks which transitions are enables.
        state_transitions: given an input, executes state transition.

    """
    def _create_subnets(self):
        """
        Creates the Subnet objects that can be used in the Petri nets.
        The created subnets are attributes of the class.

        :return:
        """
        # Kinect tracking subnet.
        # Transitions:
        #     kinect_start: initializes detection.
        #     kinect_time: time to wait to detect the surrender pose.
        #     kinect_end: human tracked.
        # Places:
        #     kinect_detection: human not tracked, waiting for detection of
        #                       surrender pose.
        matrix = np.array([[1], [0], [-1]])
        bidirectional = np.zeros(matrix.shape)
        bidirectional[1][0] = 1
        transitions = ['kinect_start', 'kinect_time', 'kinect_end']
        places = ['kinect_detection']
        times = [0, 10, 0]
        self.tracking_subnet = Subnet(matrix, bidirectional, 0, 2, -1,
                                      transitions, places, times)

        # Gesture command subnet.
        # Transitions:
        #     gesture_start: pointed object detected.
        #     gesture_correct: correct indication.
        #     gesture_wrong: wrong indication.
        #
        # Places:
        #     gesture_interpreted: pointing gesture interpreted.
        matrix = np.array([[1], [-1], [-1]])
        bidirectional = np.zeros(matrix.shape)
        transitions = ['gesture_start', 'gesture_correct', 'gesture_wrong']
        places = ['gesture_interpreted']
        times = [0, 0, 0]
        self.gesture_subnet = Subnet(matrix, bidirectional, 0, 1, 2,
                                     transitions, places, times)

        # Command execution subnet.
        # Transitions:
        #     execution_start: command arrives.
        #     execution_end: finish executing command.
        #
        # Places:
        #     executing_command: command being executed.
        matrix = np.array([[1], [-1]])
        bidirectional = np.zeros(matrix.shape)
        transitions = ['execution_start', 'execution_end']
        places = ['executing_command']
        times = [0, 0]
        self.execution_subnet = Subnet(matrix, bidirectional, 0, 1, -1,
                                       transitions, places, times)

        # Screen input subnet.
        # Transitions:
        #     screen_start: input field selected.
        #     screen_control_start: more than 1 token in 'waiting input' place.
        #     screen_control_end: finish screen control.
        #     screen_end: field completed.
        #
        # Places:
        #     waiting_input: waiting for input in the selected field.
        #     screen_control: control place.
        matrix = np.array([[1, 0], [-2, 1], [1, -1], [-1, 0]])
        bidirectional = np.zeros(matrix.shape)
        transitions = ['screen_start', 'screen_control_start',
                       'screen_control_end', 'screen_end']
        places = ['waiting_input', 'screen_control']
        times = [0, 0, 0, 0]
        self.screen_subnet = Subnet(matrix, bidirectional, 0, 3, -1,
                                    transitions, places, times)

        # Human gaze subnet.
        # Transitions:
        #     gaze_start: object detected as the human's attention focus.
        #     gaze_end: .
        #
        # Places:
        #     processing_gaze: processing the human attention focus detected.
        matrix = np.array([[1], [-1]])
        bidirectional = np.zeros(matrix.shape)
        transitions = ['gaze_start', 'gaze_end']
        places = ['processing_gaze']
        times = [0, 0]
        self.human_gaze_subnet = Subnet(matrix, bidirectional, 0, 1, -1,
                                        transitions, places, times)

        # Problem check subnet.
        # Transitions:
        #     problem_start: time without any command.
        #     problem_not_tracking: human not tracked.
        #     problem_tracking: human tracked.
        #     problem_not_pointing: human not pointing.
        #     problem_pointing: human pointing.
        #     problem_end: finish checking.
        #
        # Places:
        #     check_tracking: checking if human is being tracked.
        #     check_pointing: checking if human is pointing.
        #     check_finish: end checking, ready to finish.
        matrix = np.array([[1, 0, 0], [-1, 0, 0], [-1, 1, 0],
                           [0, -1, 1], [0, -1, 1], [0, 0, -1]])
        bidirectional = np.zeros(matrix.shape)
        transitions = ['problem_start', 'problem_not_tracking',
                       'problem_tracking', 'problem_not_pointing',
                       'problem_pointing', 'problem_end']
        places = ['check_tracking', 'check_pointing', 'check_finish']
        times = [15, 0, 0, 0, 0, 0]
        self.problem_check_subnet = Subnet(matrix, bidirectional, 0, 5, 1,
                                           transitions, places, times)

        # Finish subnet.
        # Transitions:
        #     finish_start: command for finalization detected.
        #     finish_end: finish.
        #
        # Places:
        #     finishing: ready to finish.
        matrix = np.array([[1], [-1]])
        bidirectional = np.zeros(matrix.shape)
        transitions = ['finish_start', 'finish_end']
        places = ['finishing']
        times = [0, 0]
        self.finish_subnet = Subnet(matrix, bidirectional, 0, 1, -1,
                                    transitions, places, times)

    def _concatenate_subnets(self):
        """
        Concatenates the indicated subnets, creating the Petri net attributes.

        :return:
        """

        for i in range(0, len(self._subnets)):
            self.transitions = self.transitions + self._subnets[i].transitions_names
            self.places = self.places + self._subnets[i].places_names
            self.times = self.times + self._subnets[i].transitions_times

            if i == 0:
                # Initializing the matrix with the first submatrices.
                self.matrix = self._subnets[i].incidence_matrix
                self.bi_arcs = self._subnets[i].bidirectional_arcs
            else:
                # -------- Incidence matrix -------- #
                # Matrix's top half is the last matrix and a block of zeros.
                top = np.concatenate((self.matrix,
                                      np.zeros([self.matrix.shape[0],
                                               self._subnets[i].n_places])),
                                     axis=1)

                # Matrix's bottom half is a block of zeros and the new matrix.
                bottom = np.concatenate(
                    (np.zeros([self._subnets[i].n_transitions,
                               self.matrix.shape[1]]),
                     self._subnets[i].incidence_matrix),
                    axis=1)

                # New incidence matrix.
                self.matrix = np.concatenate((top, bottom), axis=0)
                # ---------------------------------- #

                # ---- Bidirectional arcs matrix --- #
                # Matrix's top half is the last matrix and a block of zeros.
                top = np.concatenate((self.bi_arcs,
                                      np.zeros([self.bi_arcs.shape[0],
                                                self._subnets[i].n_places])),
                                     axis=1)
                # Matrix's bottom half is a block of zeros and the new matrix.
                bottom = np.concatenate(
                    (np.zeros([self._subnets[i].n_transitions,
                               self.bi_arcs.shape[1]]),
                     self._subnets[i].bidirectional_arcs),
                    axis=1)

                # New bidirectional arcs matrix.
                self.bi_arcs = np.concatenate((top, bottom), axis=0)
                # ---------------------------------- #

    def _add_transition(self, p_in, p_out, p_bi, name, time=0):
        """
        Adds a transition to the Petri net.

        :param p_in: list of places from where the new transition takes tokens.
        :param p_out: list of places where the new transition put tokens.
        :param p_bi: list of places with bidirectional arcs with the new transition.
        :param name: name of the new transition.
        :param time: if it is a time transition, the time related to it.
        :return:
        """
        matrix_line = np.zeros([1, self.matrix.shape[1]])
        bi_arcs_line = np.zeros([1, self.matrix.shape[1]])

        j = 0
        for i in p_in:
            matrix_line[i] = -w_in[j]
            j = j + 1
        j = 0
        for i in p_out:
            matrix_line[i] = w_out[j]
            j = j + 1
        self.matrix = np.concatenate((self.matrix, matrix_line), axis=0)

        for i in p_bi:
            bi_arcs_line[i] = 1
        self.bi_arcs = np.concatenate((self.bi_arcs, bi_arcs_line), axis=0)

        self.transitions.append(name)
        self.times.append(time)

    def _add_place(self, t_in, t_out, t_bi, name):
        """
        Adds a place to the Petri net.

        :param t_in: list of transitions that put tokens in the new place.
        :param t_out: list of transitions that take tokens out of the new place.
        :param t_bi: list of transitions with bidirectional arcs with the new place.
        :param name: name of the new place.
        :return:
        """
        matrix_column = np.zeros([self.matrix.shape[0], 1])
        bi_arcs_column = np.zeros([self.matrix.shape[0], 1])

        for i in t_in:
            matrix_column[i] = 1
        for i in t_out:
            matrix_column[i] = -1
        self.matrix = np.concatenate((self.matrix, matrix_column), axis=1)

        for i in t_bi:
            bi_arcs_column[i] = 1
        self.bi_arcs = np.concatenate((self.bi_arcs, bi_arcs_column), axis=1)

        self.places.append(name)

    def check_transitions(self):
        """
        Checks which transitions are enabled.

        :return: a list with the enabled transitions.
        """
        enabled = []
        for i in range(0, self.n_transitions):
            flag = 1
            for j in range(0, self.n_places):
                if self.matrix[i][j] < 0:
                    if self.state[j] < abs(self.matrix[i][j]):
                        flag = 0
                        break
                if self.bi_arcs[i][j] == 1:
                    if self.state[j] == 0:
                        flag = 0
                        break
            if flag == 1:
                enabled.append(i)

        return enabled

    def state_transition(self, input):
        """
        Given an input, executes the state transition, updating the net state.

        :param input: vector of input of the Petri net.
        :return:
        """
        enabled = self.check_transitions()
        fire = np.where(input == 1)
        if fire[1] not in enabled:
            print("Transition not enabled.")
        else:
            x = self.state + input.dot(self.matrix)
            self.state = x[0]

    def __init__(self):
        self._create_subnets()

        self.matrix = []
        self.bi_arcs = []
        self.transitions = []
        self.places = []
        self.times = []
        self.subnets = []
        self.state = []
        self.n_places = 0
        self.n_transitions = 0
        self.running = 0