import tools as tools

PROTEC_ZONE = 50


class Drone:
    def __init__(self, flightNumber, dep, arr, hDep, droneType):
        self.flightNumber = flightNumber
        self.speed = 10     # in m/s for now
        self.dep = dep
        self.arr = arr
        self.hDep = hDep
        self.type = droneType
        # A path objct that will be used to store the drone current path, previous and discretized path
        self.path_object = None

    # TODO changer cette fonction pour appartenir a model et faire find_conflict_on_edge(drone1,drone2, edge)
    def find_conflict_on_edge(self, drone, graph):
        """Detects whether the current drone is in conflict with the other given drone on any of the edges he uses"""
        self_path = self.path_object
        other_drone_path = drone.path_object
        for t in self_path.path_dict_discretized:
            (x1, y1) = self_path.path_dict_discretized[t]
            try:
                (x2, y2) = other_drone_path.path_dict_discretized[t]
                if tools.distance(x1, y1, x2, y2) < PROTEC_ZONE:
                    edge = self.find_current_edge(t, graph)
                    edge2 = drone.find_current_edge(t, graph)
                    if edge == edge2 or edge == (edge2[1], edge2[0]):
                        return t
            except KeyError:
                pass
        return None

    def find_conflict_on_node(self, other_drone, model):
        self_path = self.path_object
        other_drone_path = other_drone.path_object
        protection_area = model.protection_area
        for t in self_path.path_dict:
            node = self_path.path_dict[t]
            other_drone_nodes = list(other_drone_path.path_dict.values())
            if node in other_drone_nodes:
                i = other_drone_nodes.index(node)
                t_other_drone = list(other_drone_path.path_dict.keys())[i]
                # Check who goes first and that it had time to leave entirely
                if t > t_other_drone:
                    if t - t_other_drone < protection_area/self.speed:
                        return t_other_drone
                if t_other_drone > t:
                    if t_other_drone - t < protection_area/other_drone.speed:
                        return t
        return None

    def find_current_edge(self, t, graph):
        """Finds the edge of the drone at given time t in seconds."""
        fromNode = ''
        selfPath = self.path_object
        for tt in selfPath.path_dict:
            if tt <= t:
                fromNode = selfPath.path_dict[tt]
            if tt > t:
                return fromNode, selfPath.path_dict[tt]
            