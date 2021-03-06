class Drone:
    def __init__(self, flightNumber, dep, arr, hDep, droneType):
        self.flight_number = flightNumber
        self.speeds_dict = {"cruise": 15.4333, "turn1": 5.144, "turn2": 2.5722, "turn3": 1.02889}
        self.accel_max = 3
        self.cruise_speed = 15.4333     # in m/s for now
        # Todo ajouter les turn speed pour diff angle de virage
        self.turn_speed = 5.144
        self.braking_distance = 30
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
                if from_node != '':
                    return from_node, self.path_object.path_dict[tt]
        # if conflict time is before take off time or beetween first and second node
        #TODO est ce que c'est ok ?
        # print("Conflict before take off time")
        return self.path_object.path[0], self.path_object.path[1]

            