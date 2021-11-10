import Drone as dr
import math
import tools


class Model:
    def __init__(self, graph, protection_area, dt=5, drones=[]):
        self.graph = init_graph(graph)
        self.graph_dual = None
        self.time = 0   # keep track of the time in seconds
        self.timeInterval = dt
        self.droneList = drones
        self.protection_area = protection_area

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

    def find_conflicts(self, graph):
        """Detect conflicts on the graph by using the distances between each drones at any time"""
        conflict_list = []
        for i, drone in enumerate(self.droneList):
            if i != len(self.droneList)-1:
                for j in range(i+1, len(self.droneList)):
                    t_edge = self.find_conflict_on_edge(drone, self.droneList[j], graph)
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

    def find_conflict_on_edge(self, drone1, drone2, graph):
        """Detects whether the current drone is in conflict with the other given drone on any of the edges he uses"""
        drone1_path = drone1.path_object
        drone2_path = drone2.path_object
        # TODO Pas solide parce que ça marche que sur le pas de temps défini, les points de temps en dehors des pas de
        #  temps exacts son pas analysés
        for t in drone1_path.path_dict_discretized:
            (x1, y1) = drone1_path.path_dict_discretized[t]
            if t in drone2_path.path_dict_discretized:
                (x2, y2) = drone2_path.path_dict_discretized[t]
                if tools.distance(x1, y1, x2, y2) < self.protection_area:
                    edge = drone1.find_current_edge(t, graph)
                    edge2 = drone2.find_current_edge(t, graph)
                    if edge == edge2 or edge == (edge2[1], edge2[0]):
                        return t
            else:
                pass
        return None

    def find_conflict_on_node(self, drone1, drone2):
        drone1_path = drone1.path_object
        drone2_path = drone2.path_object
        protection_area = self.protection_area
        for t in drone1_path.path_dict:
            node = drone1_path.path_dict[t]
            other_drone_nodes = list(drone2_path.path_dict.values())
            if node in other_drone_nodes:
                i = other_drone_nodes.index(node)
                t_other_drone = list(drone2_path.path_dict.keys())[i]
                # Check who goes first and that it had time to leave entirely
                if t > t_other_drone:
                    if t - t_other_drone < protection_area/drone1.speed:
                        return t_other_drone
                if t_other_drone > t:
                    if t_other_drone - t < protection_area/drone2.speed:
                        return t
        return None


def init_graph(graph):
    for edge in graph.edges:
        edge_dict = graph.edges[edge]
        edge_dict['open'] = True
        edge_dict['length'] = float(edge_dict['length'])
        edge_dict['geometry'] = edge_dict['geometry'].strip(')').split('(')[1].split()
        for i, coord in enumerate(edge_dict['geometry']):
            edge_dict['geometry'][i] = float(coord.strip(','))
        # graph.edges[edge]['maxspeed'] = float(graph.edges[edge]['maxspeed'])
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


