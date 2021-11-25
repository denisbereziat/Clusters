import Drone as dr
import math
import tools


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
                    t_edge = self.find_conflict_on_edge(drone, self.droneList[j])
                    t_node = self.find_conflict_on_node(drone, self.droneList[j])
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

    def find_conflict_on_edge(self, drone1, drone2):
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
                # Going the same way on the edge
                if node1_1 == node2_1 and node2_2 == node2_2:
                    if t1_1 < t2_1 and not t1_2 < t2_2:
                        return t2_1
                    if t2_1 < t1_1 and not t2_2 < t1_2:
                        return t1_1
                # Going opposite way on the edge
                if node1_1 == node2_2 and node1_2 == node2_1:
                    if tools.intersection([t1_1, t1_2], [t2_1, t2_2]) is not None:
                        return max(t1_1, t2_1)

    def find_conflict_on_node(self, drone1, drone2):
        """Check all the nodes of the drones path and if there is conflict on any, there is a conflict if the node
        is used twice without sufficient time in between for the first drone to have gone a sufficient distance
        away, this distance is the protection_area"""
        drone1_path = drone1.path_object
        drone2_path = drone2.path_object
        protection_area = self.protection_area
        for t_drone1 in drone1_path.path_dict:
            for t_drone2 in drone2_path.path_dict:
                if drone2_path.path_dict[t_drone2] == drone1_path.path_dict[t_drone1]:

                    if t_drone1 > t_drone2:
                        # We need the drone next time stamp to know his speed after the node
                        drone1_time_stamps = sorted(drone1_path.path_dict.keys())
                        if not t_drone1 == drone1_time_stamps[-1]:
                            drone1_next_time_stamp = drone1_time_stamps[drone1_time_stamps.index(t_drone1) + 1]
                        else:
                            drone1_next_time_stamp = t_drone1
                        drone1_speed = self.get_current_drone_speed(drone1_next_time_stamp, drone1)
                        if t_drone1 - t_drone2 < protection_area/drone1_speed:
                            return t_drone1

                    if t_drone2 > t_drone1:
                        # We need the drone next time stamp to know his speed after the node
                        drone2_time_stamps = sorted(drone2_path.path_dict.keys())
                        if not t_drone2 == drone2_time_stamps[-1]:
                            drone2_next_time_stamp = drone2_time_stamps[drone2_time_stamps.index(t_drone2) + 1]
                        else:
                            drone2_next_time_stamp = t_drone2
                        drone2_speed = self.get_current_drone_speed(drone2_next_time_stamp, drone2)
                        if t_drone2 - t_drone1 < protection_area/drone2_speed:
                            return t_drone2
                #
                # drone2_nodes = list(drone2_path.path_dict.values())
                # if node in drone2_nodes:
                #     node_index_drone2 = drone2_nodes.index(node)
                #     t_drone2 = list(drone2_path.path_dict.keys())[node_index_drone2]
                #     # Check who goes first and that it had time to leave entirely
                #     # To check if the drone had time to leave far enough we need the drone speed after passing the node
                #     # So we need to know if the node was a turning point or if the next one is
                #     # Check if drone1 is turning :
                #     # If either the last node or this node is a turning point, he is at turn speed until the node
                #     # TODO C'est la exit speed la qu'il faut
        return None

    def get_current_drone_speed(self, current_t, drone):
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

                # if node_index > 1:
                #     current_edge = (path_dict[node_index - 1], path_dict[node_time_list[node_index]])
                # if len(path_dict) - 1 > node_index + 2:
                #     next_edge = (path_dict[node_time_list[node_index + 1]], path_dict[node_time_list[node_index + 2]])
                # if node_index > 0:
                #     previous_edge = (path_dict[node_time_list[node_index-1]], path_dict[current_t])
                # if len(path_dict) - 1 > index + 1:
                #     current_edge = (path_dict[current_t], path_dict[node_time_list[index + 1]])
                #     if len(path_dict) - 1 > index + 2:
                #         next_edge = (path_dict[node_time_list[index + 1]], path_dict[node_time_list[index + 2]])
                # if index > 0:
                #     previous_edge = (path_dict[node_time_list[index-1]], path_dict[current_t])
        # if current_edge is not None and previous_edge is not None:
        #     if self.graph_dual.edges[previous_edge, current_edge]["is_turn"]:
        #         return drone.turn_speed
        # if current_edge is not None and next_edge is not None:
        #     if self.graph_dual.edges[current_edge, next_edge]["is_turn"]:
        #         return drone.turn_speed
        # return drone.cruise_speed









def init_graph(graph):
    for edge in graph.edges:
        edge_dict = graph.edges[edge]
        edge_dict['open'] = True
        edge_dict['length'] = float(edge_dict['length'])
        edge_dict['geometry'] = edge_dict['geometry'].strip(')').split('(')[1].split()
        for i, coord in enumerate(edge_dict['geometry']):
            edge_dict['geometry'][i] = float(coord.strip(','))
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


