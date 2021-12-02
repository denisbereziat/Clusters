import Drone as dr
import math
import tools
import networkx as nx


class Model:
    """Class used to store the drones objects, the primal graph, the dual graph, safety parameters
     and find conflicts."""
    def __init__(self, graph, protection_area, dt=5, drones=[], initial_constraints=None):
        self.graph = init_graph(graph)
        self.graph_dual = None
        self.timeInterval = dt
        self.droneList = drones
        self.protection_area = protection_area
        # Initial constraints contains the constraints caused by the currently
        # flying drones when the model is initialised
        self.initial_constraints_dict = initial_constraints
        # turning_edges store the edges that are considered as turns
        self.turning_edges = dict()
        # Count Astar iterations for testing
        self.countAstar = 0

    def add_drone(self, drone):
        self.droneList.append(drone)

    def set_graph_dual(self, graph_dual):
        self.graph_dual = graph_dual

    def add_drones_from_file(self, filename, time):
        """Extract the information of drones from the given file and add them to the model"""
        time_in_seconds = float(time[0:2])*3600 + float(time[3:5])*60 + float(time[6:8])
        with open(filename, 'r') as f:
            for line in f:
                line = line.strip().split('\t')
                dep_coordinates = line[4].strip('()').split(',')
                arr_coordinates = line[5].strip('()').split(',')
                dep = get_closest_node(float(dep_coordinates[1]), float(dep_coordinates[0]), self.graph)
                arr = get_closest_node(float(arr_coordinates[1]), float(arr_coordinates[0]), self.graph)
                deposit_time_in_seconds = float(line[0][0:2])*3600 + float(line[0][3:5])*60 + float(line[0][6:8])
                # print("Deposit time :", line)
                # print("Deposit time :", deposit_time_in_seconds)
                dep_time = float(line[3][0:2])*3600 + float(line[3][3:5])*60 + float(line[3][6:8])
                drone = dr.Drone(line[1], dep, arr, dep_time, line[2])
                if deposit_time_in_seconds <= time_in_seconds:
                    self.add_drone(drone)

    def find_conflicts(self):
        """Detect conflicts on the graph by using the distances between each drones at any time"""
        conflict_list = []
        for i, drone in enumerate(self.droneList):
            if i != len(self.droneList)-1:
                for j in range(i+1, len(self.droneList)):
                    t_edge = find_conflict_on_edges(drone, self.droneList[j])
                    t_node = find_conflict_on_nodes(drone, self.droneList[j], self.protection_area, self.graph_dual,
                                                    self.graph)
                    if t_edge is not None:
                        if t_node is not None:
                            if t_edge <= t_node:
                                conflict_list.append((i, j, t_edge))
                            else:
                                conflict_list.append((i, j, t_node))
                    elif t_node is not None:
                        conflict_list.append((i, j, t_node))
        return conflict_list

    def open_all_nodes(self):
        for edge in self.graph.edges:
            self.graph.edges[edge]['open'] = True


def find_conflict_on_edges(drone1, drone2):
    """Check all edges taken by the drones and if there is any conflicts. There is conflict on an edge if it is
    taken both way at the same time by two drones or if is taken the same way but the 2 drones don't exist on the
    same order, meaning they passed each other"""
    drone1_path = drone1.path_object
    drone2_path = drone2.path_object

    # TODO trouver la maniere la plus rapide de faire : checker si edge correspondent d'abord ? ou t ?
    times_drone_1 = list(drone1_path.path_dict.keys())
    times_drone_2 = list(drone2_path.path_dict.keys())
    times_drone_1.sort()
    times_drone_2.sort()
    for index_t1 in range(len(times_drone_1) - 1):
        t1_1, t1_2 = times_drone_1[index_t1], times_drone_1[index_t1 + 1]
        for index_t2 in range(len(times_drone_2) - 1):
            t2_1, t2_2 = times_drone_2[index_t2], times_drone_2[index_t2 + 1]
            node1_1, node1_2 = drone1_path.path_dict[t1_1], drone1_path.path_dict[t1_2]
            node2_1, node2_2 = drone2_path.path_dict[t2_1], drone2_path.path_dict[t2_2]
            conflict = check_conflict_on_edge(node1_1, node1_2, node2_1, node2_2, t1_1, t1_2, t2_1, t2_2)
            if conflict is not None:
                return conflict


def check_conflict_on_edge(node1_1, node1_2, node2_1, node2_2, t1_1, t1_2, t2_1, t2_2):
    # Going the same way on the edge
    if node1_1 == node2_1 and node2_2 == node2_2:
        if t1_1 < t2_1 and not t1_2 < t2_2:
            return t2_2
        if t2_1 < t1_1 and not t2_2 < t1_2:
            return t1_2
    # Going opposite way on the edge
    if node1_1 == node2_2 and node1_2 == node2_1:
        if tools.intersection([t1_1, t1_2], [t2_1, t2_2]) is not None:
            return max(t1_2, t2_2)


def find_conflict_on_nodes(drone1, drone2, protection_area, graph_dual, graph):
    """Check all the nodes of the drones path and if there is conflict on any, there is a conflict if the node
    is used twice without sufficient time in between for the first drone to have gone a sufficient distance
    away, this distance is the protection_area"""

    drone1_path = drone1.path_object
    drone2_path = drone2.path_object

    drone1_time_stamps = sorted(drone1_path.path_dict.keys())
    drone2_time_stamps = sorted(drone2_path.path_dict.keys())

    for t_drone1 in drone1_time_stamps:
        for t_drone2 in drone2_time_stamps:
            if drone2_path.path_dict[t_drone2] == drone1_path.path_dict[t_drone1]:

                drone1_speed_after = drone1.cruise_speed
                drone2_speed_after = drone2.cruise_speed

                # Index of both drones in their path time stamps
                d1_index = drone1_time_stamps.index(t_drone1)
                d2_index = drone2_time_stamps.index(t_drone2)

                # We need to know which drone enter the node first and the exit speed
                # If the node was a turning point, speed is turn speed
                # If the next node is a turning point and dist < braking_dist + safety_dist, speed is turn speed
                # Else we consider the drone goes away from the node at cruise_speed

                d1_node = drone1_path.path_dict[t_drone1]
                d2_node = drone2_path.path_dict[t_drone2]

                d1_node_minus1, d1_node_plus1, d1_node_plus2 = None, None, None
                d2_node_minus1, d2_node_plus1, d2_node_plus2 = None, None, None

                if d1_index > 0:
                    d1_node_minus1 = drone1_path.path_dict[drone1_time_stamps[d1_index-1]]
                if d2_index > 0:
                    d2_node_minus1 = drone2_path.path_dict[drone2_time_stamps[d2_index-1]]
                if d1_index+1 <= len(drone1_time_stamps) - 1:
                    d1_node_plus1 = drone1_path.path_dict[drone1_time_stamps[d1_index+1]]
                if d2_index+1 <= len(drone2_time_stamps) - 1:
                    d2_node_plus1 = drone2_path.path_dict[drone2_time_stamps[d2_index+1]]
                if d1_index+2 <= len(drone1_time_stamps) - 1:
                    d1_node_plus2 = drone1_path.path_dict[drone1_time_stamps[d1_index+2]]
                if d2_index+2 <= len(drone2_time_stamps) - 1:
                    d2_node_plus2 = drone2_path.path_dict[drone2_time_stamps[d2_index+2]]

                # Was the node a turning point ?
                if d1_node_minus1 is not None and d1_node_plus1 is not None:
                    if graph_dual.edges[(d1_node_minus1, d1_node), (d1_node, d1_node_plus1)]["is_turn"]:
                        drone1_speed_after = drone1.turn_speed
                # Same for drone2
                if d2_node_minus1 is not None and d2_node_plus1 is not None:
                    if graph_dual.edges[(d2_node_minus1, d2_node), (d2_node, d2_node_plus1)]["is_turn"]:
                        drone2_speed_after = drone2.turn_speed

                # Is the next node a turning point and is it far enough ?
                if d1_node_plus1 is not None and d1_node_plus2 is not None:
                    if graph_dual.edges[(d1_node, d1_node_plus1), (d1_node_plus1, d1_node_plus2)]["is_turn"]:
                        # Check that the distance between the 2 nodes is sufficient
                        length = graph.edges[d1_node, d1_node_plus1]['length']
                        # Dist need to be more than braking distance + safety
                        if length < protection_area + drone1.braking_distance:
                            drone1_speed_after = drone1.turn_speed
                # Same process for the second drone
                if d2_node_plus1 is not None and d2_node_plus2 is not None:
                    if graph_dual.edges[(d2_node, d2_node_plus1), (d2_node_plus1, d2_node_plus2)]["is_turn"]:
                        # Check that the distance between the 2 nodes is sufficient
                        length = graph.edges[d2_node, d2_node_plus1]['length']
                        # Dist need to be more than braking distance + safety
                        if length < protection_area + drone2.braking_distance:
                            drone2_speed_after = drone2.turn_speed

                # Check for conflict :
                conflict_time = check_conflict_on_node(t_drone1, t_drone2, drone1_speed_after, drone2_speed_after,
                                                       protection_area)
                if conflict_time is not None:
                    return conflict_time
    return None
    #
    #             # We need the drones next time stamps to know their speed after the node
    #             # Drone speed when reaching the node
    #             # d1_speed_before = get_current_drone_speed(drone1_time_stamps[t_drone1], drone1)
    #             if not t_drone1 == drone1_time_stamps[-1]:
    #                 drone1_next_time_stamp = drone1_time_stamps[drone1_time_stamps.index(t_drone1) + 1]
    #             else:
    #                 drone1_next_time_stamp = t_drone1
    #             d1_speed_after = get_current_drone_speed(drone1_next_time_stamp, drone1)
    #             # d2_speed_before = get_current_drone_speed(drone2_time_stamps[t_drone2], drone2)
    #             if not t_drone2 == drone2_time_stamps[-1]:
    #                 drone2_next_time_stamp = drone2_time_stamps[drone2_time_stamps.index(t_drone2) + 1]
    #             else:
    #                 drone2_next_time_stamp = t_drone2
    #             d2_speed_after = get_current_drone_speed(drone2_next_time_stamp, drone2)
    #             conflict_time = check_conflict_on_node(t_drone1, t_drone2, d1_speed_after, d2_speed_after, protection_area)
    #             # Check for conflict :
    #             if conflict_time is not None:
    #                 return conflict_time
    # return None


def check_conflict_on_node(t_drone1, t_drone2, d1_speed_after, d2_speed_after, protection_area):
    """Check for conflict on a node in the constrained airspace"""
    if t_drone1 > t_drone2:
        if t_drone1 - t_drone2 < protection_area / d1_speed_after:
            return t_drone1
    if t_drone2 > t_drone1:
        if t_drone2 - t_drone1 < protection_area / d2_speed_after:
            return t_drone2



def get_current_drone_speed(current_t, drone):
    """Return the current drone speed at the specified node"""
    current_edge, previous_edge, next_edge = None, None, None
    path_dict = drone.path_object.path_dict
    node_time_list = list(path_dict.keys())
    node_time_list.sort()
    #TODO cas particulier pour les noeuds d'arrive
    for node_index, t in enumerate(node_time_list):
        if current_t == t:
            if node_index == 0:
                return drone.path_object.speed_time_stamps[0][1]
            else:
                return drone.path_object.speed_time_stamps[node_index-1][1]


def init_graph(graph : nx.Graph):
    #TODO maintenant qu'on a qu'un graph pas besoin de l'init plein de fois
    for edge in graph.edges:
        edge_dict = graph.edges[edge]
        edge_dict['open'] = True
        edge_dict['length'] = float(edge_dict['length'])
        try:
            edge_dict['geometry'] = edge_dict['geometry'].strip(')').split('(')[1].split()
            for i, coord in enumerate(edge_dict['geometry']):
                edge_dict['geometry'][i] = float(coord.strip(','))
        except AttributeError:
            # Exception here if the list has already been processed
            pass
    for node in graph.nodes:
        node_dict = graph.nodes[node]
        node_dict['x'] = float(node_dict['x'])
        node_dict['y'] = float(node_dict['y'])
    return graph        
                

def get_closest_node(x, y, graph):
    """For a given set of coordinates, gives the closest node in the graph."""
    closest_node = None
    min_dist = 500000
    
    for node in graph.nodes():
        dist = math.sqrt((x-graph.nodes[node]['x'])**2+(y-graph.nodes[node]['y'])**2)
        if dist == 0:
            return node
        if dist < min_dist:
            min_dist = dist
            closest_node = node
    
    return closest_node


