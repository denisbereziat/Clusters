class Drone:
    def __init__(self, flightNumber, dep, arr, hDep, droneType):
        self.flight_number = flightNumber
        self.speed = 15.4333     # in m/s for now
        self.turn_speed = 5.1444
        self.dep = dep
        self.arr = arr
        self.dep_time = hDep
        self.type = droneType
        # A path object that will be used to store the drone current path, previous and discretized path
        self.path_object = None

    def find_current_edge(self, current_t, graph):
        """Finds the edge of the drone at given time t in seconds."""
        from_node = ''
        for tt in self.path_object.path_dict:
            if tt <= current_t:
                from_node = self.path_object.path_dict[tt]
            if tt > current_t:
                return from_node, self.path_object.path_dict[tt]
            