class Subnet:
    """
    Subnet is a Petri net that has a specific function. When used as a block in
    a more complex Petri net, the subnet can be seen in a simplified way that
    contains one transition to start, one place indicating that the process is
    running and one transition to finish. Sometimes it can have one extra
    transition with another possible output.

    Attributes:
        incidence_matrix: incidence matrix of the subnet.
        bidirectional_arcs: matrix indicating bidirectional arcs.
        start_transition: index of the start transition in the matrix.
        end_transition: index of the end transition in the matrix.
        extra_transition: index of the extra output transition in the matrix.
        transitions_names: names of transitions.
        places_names: names of places.
        n_transitions: number of transitions.
        n_places: number of places
    """

    def __init__(self, matrix, bidirectional_arcs,
                 start, end, extra,
                 transitions_names, places_names, times):
        """
        Constructor method.

        :param matrix: incidence matrix.
        :param bidirectional_arcs: a list with the bidirectional arcs.
        :param start: index of the start transition.
        :param end: index of the end transition.
        :param extra: index of the extra transition. -1 if is not used.
        :param transitions_names: names of transitions.
        :param places_names: names of places.
        """
        self.incidence_matrix = matrix
        self.bidirectional_arcs = bidirectional_arcs
        self.start_transition = start
        self.end_transition = end
        self.extra_transition = extra
        self.transitions_names = transitions_names
        self.places_names = places_names

        _matrix_size = matrix.shape
        self.n_transitions = _matrix_size[0]
        self.n_places = _matrix_size[1]
        self.transitions_times = times
