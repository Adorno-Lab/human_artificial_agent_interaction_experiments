from include.PetriNet import *
import numpy as np
import copy as cp


class ExPhaseTwo(PetriNet):

    def _create(self):
        # Including subnets.
        self._subnets = [self.screen_subnet]

        # Creating a list with the indexes of the first transition of each
        # submatrix.
        subnet_index = [0]
        for i in range(1, len(self._subnets)):
            subnet_index.append(
                subnet_index[i - 1] + self._subnets[i - 1].n_transitions)

        self._concatenate_subnets()

        # Indexes of the beginning of external transitions and places.
        t_ext_index = self.matrix.shape[0]
        p_ext_index = self.matrix.shape[1]

        # ------------------------ Transitions ------------------------ #
        # To define a new transition, set its in and out places and the
        # bidirectional arcs. Then call the add_transition function.

        # 0) Start counting phase:
        in_places = []
        out_places = []
        bi_places = []
        self._add_transition(in_places, out_places, bi_places,
                             'start_counting_phase')

        # 1) Start command:
        # Explicit command to start the phase.
        in_places = []
        out_places = []
        bi_places = []
        self._add_transition(in_places, out_places, bi_places,
                             'start_command')

        # 2) Enable fields:
        # Enable entry fields in the screen application.
        in_places = []
        out_places = []
        bi_places = []
        self._add_transition(in_places, out_places, bi_places,
                             'enable_fields', 5)

        # 3) Time limit:
        # Phase limit of time reached.
        in_places = []
        out_places = []
        bi_places = []
        self._add_transition(in_places, out_places, bi_places,
                             'time_limit_counting', 240)

        # 4) Completed counting:
        # Counting was completed by the virtual agent.
        in_places = []
        out_places = []
        bi_places = []
        self._add_transition(in_places, out_places, bi_places,
                             'counting_completed')

        # 5) Counting not finished:
        # Human did not finish the counting yet.
        in_places = []
        out_places = []
        bi_places = []
        self._add_transition(in_places, out_places, bi_places,
                             'counting_not_finished')

        # 6) Counting finished:
        # Human finished the counting.
        in_places = []
        out_places = []
        bi_places = []
        self._add_transition(in_places, out_places, bi_places,
                             'counting_finished')

        # 7) Repeat instructions:
        # Repeat instructions after a while without screen commands.
        in_places = []
        out_places = []
        bi_places = []
        self._add_transition(in_places, out_places, bi_places,
                             'repeat_instructions', 30)

        # -------------------------- Places --------------------------- #
        # To define a new place, set its in and out transitions and the
        # bidirectional arcs. Then call the add_place function.

        # 0) Waiting start:
        # Waiting for the start command.
        in_transitions = [t_ext_index]
        out_transitions = [t_ext_index + 1]
        bi_transitions = []
        self._add_place(in_transitions, out_transitions, bi_transitions,
                        'waiting_start')

        # 1) Phase instructions:
        # Instructions about phase.
        in_transitions = [t_ext_index + 1]
        out_transitions = [t_ext_index + 2]
        bi_transitions = []
        self._add_place(in_transitions, out_transitions, bi_transitions,
                        'phase_instructions')
                
        # 2) Waiting selection:
        # Waiting the selection of an input field.
        in_transitions = [t_ext_index + 2,
                          t_ext_index + 5]
        out_transitions = [subnet_index[0] + self._subnets[0].end_transition,
                           t_ext_index + 3]
        bi_transitions = [subnet_index[0] + self._subnets[0].start_transition,
                          t_ext_index + 7]
        self._add_place(in_transitions, out_transitions, bi_transitions,
                        'waiting_selection')
        
        # 3) Complete counting:
        # Agent virtual should complete the counting.
        in_transitions = [t_ext_index + 3]
        out_transitions = [t_ext_index + 4]
        bi_transitions = []
        self._add_place(in_transitions, out_transitions, bi_transitions,
                        'complete_counting')
        
        # 4) Check counting:
        # Check if the counting is finished or not.
        in_transitions = [subnet_index[0] + self._subnets[0].end_transition]
        out_transitions = [t_ext_index + 5,
                           t_ext_index + 6]
        bi_transitions = []
        self._add_place(in_transitions, out_transitions, bi_transitions,
                        'check_counting')

        # 5) End counting phase:
        # Counting phase finished.
        in_transitions = [t_ext_index + 4,
                          t_ext_index + 6]
        out_transitions = []
        bi_transitions = []
        self._add_place(in_transitions, out_transitions, bi_transitions,
                        'end_counting_phase')

        # ------------------------------------------------------------- #

        # Initial state.
        self.state = np.zeros(self.matrix.shape[1])

        self.n_places = len(self.places)
        self.n_transitions = len(self.transitions)
        self.running = 0


    def __init__(self):
        super().__init__()
        self._create()