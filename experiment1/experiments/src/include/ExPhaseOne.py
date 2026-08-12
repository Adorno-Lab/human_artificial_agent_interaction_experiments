from include.PetriNet import *
import numpy as np
import copy as cp


class ExPhaseOne(PetriNet):

    def _create(self):
        # Including subnets.
        self._subnets = [self.tracking_subnet,
                         self.problem_check_subnet,
                         cp.deepcopy(self.tracking_subnet),
                         self.gesture_subnet]

        # Changing names of the second tracking subnet.
        for i in range(0, self._subnets[2].n_transitions):
            self._subnets[2].transitions_names[i] = \
                self._subnets[2].transitions_names[i] + "1"
        for i in range(0, self._subnets[2].n_places):
            self._subnets[2].places_names[i] = \
                self._subnets[2].places_names[i] + "1"

        # Creating a list with the indexes of the first transition of each
        # submatrix.
        subnet_index = [0]
        for i in range(1, len(self._subnets)):
            subnet_index.append(subnet_index[i-1] + self._subnets[i-1].n_transitions)

        self._concatenate_subnets()

        # Indexes of the beginning of external transitions and places.
        t_ext_index = self.matrix.shape[0]
        p_ext_index = self.matrix.shape[1]

        # ------------------------ Transitions ------------------------ #
        # To define a new transition, set its in and out places and the
        # bidirectional arcs. Then call the add_transition function.

        # 0) Start password phase:
        in_places = []
        out_places = []
        bi_places = []
        self._add_transition(in_places, out_places, bi_places,
                             'start_password_phase')

        # 1) Detected:
        # The human is already detected at the beginning of the phase.
        in_places = []
        out_places = []
        bi_places = []
        self._add_transition(in_places, out_places, bi_places,
                             'detected')

        # 2) Start accepting commands:
        # Enabling the processing of gesture commands.
        in_places = []
        out_places = []
        bi_places = []
        self._add_transition(in_places, out_places, bi_places,
                             'start_accepting', 4)

        # 3) Time limit:
        # Phase limit of time reached.
        in_places = []
        out_places = []
        bi_places = []
        self._add_transition(in_places, out_places, bi_places,
                             'time_limit_password', 240)

        # 4) Completed password:
        # Password was completed by the virtual agent.
        in_places = []
        out_places = []
        bi_places = []
        self._add_transition(in_places, out_places, bi_places,
                             'password_completed')

        # 5) Error limit:
        # Limit of errors reached.
        in_places = []
        out_places = []
        bi_places = []
        self._add_transition(in_places, out_places, bi_places,
                             'error_limit')

        # 6) Wrong not limit:
        # Wrong indication but error limit not reached.
        in_places = []
        out_places = []
        bi_places = []
        self._add_transition(in_places, out_places, bi_places,
                             'wrong_not_limit', 2)

        # 7) Waiting after correct:
        # Waiting time after a correct indication.
        in_places = []
        out_places = []
        bi_places = []
        self._add_transition(in_places, out_places, bi_places,
                             'correct_wait', 2)

        # 8) Password finished:
        # Human finished the password.
        in_places = []
        out_places = []
        bi_places = []
        self._add_transition(in_places, out_places, bi_places,
                             'password_finished')

        # 9) Stop pointing:
        # Human stopped pointing after a correct indication.
        in_places = []
        out_places = []
        bi_places = []
        self._add_transition(in_places, out_places, bi_places,
                             'stop_pointing')

        # -------------------------- Places --------------------------- #
        # To define a new place, set its in and out transitions and the
        # bidirectional arcs. Then call the add_place function.

        # 0) Greeting:
        # Virtual agent should introduce itself.
        in_transitions = [t_ext_index]
        out_transitions = [subnet_index[0] + self._subnets[0].start_transition,
                           t_ext_index + 1]
        bi_transitions = []
        self._add_place(in_transitions, out_transitions, bi_transitions,
                        'greeting')

        # 1) Show password:
        # Show the password in the screen.
        in_transitions = [subnet_index[0] + self._subnets[0].end_transition,
                          t_ext_index + 1,
                          t_ext_index + 5]
        out_transitions = [t_ext_index + 2]
        bi_transitions = []
        self._add_place(in_transitions, out_transitions, bi_transitions,
                        'password')

        # 2) Accepting gestures:
        # Waiting for gesture commands.
        in_transitions = [t_ext_index + 2,
                          subnet_index[1] + self._subnets[1].end_transition,
                          subnet_index[2] + self._subnets[2].end_transition,
                          t_ext_index + 6,
                          t_ext_index + 7,
                          t_ext_index + 9]
        out_transitions = [subnet_index[1] + self._subnets[1].start_transition,
                           subnet_index[3] + self._subnets[3].start_transition,
                           t_ext_index + 3]
        bi_transitions = []
        self._add_place(in_transitions, out_transitions, bi_transitions,
                        'accepting')

        # 3) Lost human:
        # Human detection failed.
        in_transitions = [subnet_index[1] + self._subnets[1].extra_transition]
        out_transitions = [subnet_index[2] + self._subnets[2].start_transition]
        bi_transitions = []
        self._add_place(in_transitions, out_transitions, bi_transitions,
                        'lost_human')

        # 4) Complete password:
        # Agent virtual should complete the password.
        in_transitions = [t_ext_index + 3]
        out_transitions = [t_ext_index + 4]
        bi_transitions = []
        self._add_place(in_transitions, out_transitions, bi_transitions,
                        'complete_password')

        # 5) Check errors:
        # Check the number of errors.
        in_transitions = [subnet_index[3] + self._subnets[3].extra_transition]
        out_transitions = [t_ext_index + 5,
                           t_ext_index + 6]
        bi_transitions = []
        self._add_place(in_transitions, out_transitions, bi_transitions,
                        'check_errors')

        # 6) Check password:
        # Check if the password is finished or not.
        in_transitions = [subnet_index[3] + self._subnets[3].end_transition]
        out_transitions = [t_ext_index + 7,
                           t_ext_index + 8,
                           t_ext_index + 9]
        bi_transitions = []
        self._add_place(in_transitions, out_transitions, bi_transitions,
                        'check_password')

        # 7) End password phase:
        # Password phase finished.
        in_transitions = [t_ext_index + 4,
                          t_ext_index + 8]
        out_transitions = []
        bi_transitions = []
        self._add_place(in_transitions, out_transitions, bi_transitions,
                        'end_password_phase')

        # ------------------------------------------------------------- #

        # Initial state.
        self.state = np.zeros(self.matrix.shape[1])

        self.n_places = len(self.places)
        self.n_transitions = len(self.transitions)
        self.running = 0

    def __init__(self):
        super().__init__()
        self._create()
