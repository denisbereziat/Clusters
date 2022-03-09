import shapely.geometry

import Drone
import Drone as dr
import math
import tools
import networkx as nx
import csv
import dual_graph
import geopandas
from shapely.geometry import Point

import matplotlib.pyplot as plt

protection_area = 32
vertical_protection = 7.62  # 25 ft
nb_FL = 16
delay_max = 100
FL_sep = 9.14  # in m
temps_sep_vertiport = 5
turn_angle_mini = Drone.angle_intervals[0]


class Model:
    """Class used to store the drones objects, the primal graph, the dual graph, safety parameters
     and find conflicts."""
    def __init__(self, graph, dt=5, drones=[], initial_constraints=None):
        self.graph = init_graph(graph)
        self.graph_dual = None
        self.timeInterval = dt
        self.droneList = drones
        self.total_drone_list = []
        self.total_drone_dict = dict()
        self.drone_order = []
        self.protection_area = protection_area
        self.vertical_protection = vertical_protection
        self.nb_FL = nb_FL
        self.delay_max = delay_max
        self.FL_sep = FL_sep
        self.temps_sep_vertiport = temps_sep_vertiport
        # Initial constraints contains the constraints caused by the currently
        # flying drones when the model is initialised
        self.initial_constraints_dict = initial_constraints
        # turning_edges store the edges that are considered as turns
        self.turning_edges = dict()
        # Count Astar iterations for testing
        self.countAstar = 0
        self.hash_map = None
        self.generation_params = None

    def add_drone(self, drone):
        self.droneList.append(drone)

    def set_graph_dual(self, graph_dual):
        self.graph_dual = graph_dual

    def add_drones_from_file(self, filename, time):
        if filename[-4:] == ".csv":
            self.add_drones_from_csv_file(filename, time)
        else:
            self.add_drones_from_file_old(filename, time)

    def add_drones_from_file_old(self, filename, time):
        """Extract the information of drones from the given file and add them to the model"""
        print("Loading from old format")
        if time is not None:
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
        else:
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
                    self.add_drone(drone)

    def add_drones_from_csv_file(self, filename, time):
        """Extract the information of drones from the given file and add them to the model"""
        print("Loading from CSV file")
        if time is None:
            time = math.inf
        with open(filename, newline='') as csv_file:
            reader = csv.reader(csv_file, delimiter=',', quotechar='|')
            for line in reader:
                # print(line)
                deposit_time = float(line[0][0:2])*3600 + float(line[0][3:5])*60 + float(line[0][6:8])
                # print(time, deposit_time)
                if deposit_time > time:
                    continue
                drone_model = line[2]
                dep_vertiport_coordinates = (float(line[4].strip("\"(")), float(line[5].strip("\")")))
                arr_vertiport_coordinates = (float(line[6].strip("\"(")), float(line[7].strip("\")")))
                hash_nodes, hash_edges, min_x, min_y, x_step, y_step, resolution = self.hash_map

                # Find departure edge and node of drone
                x_dep, y_dep = dep_vertiport_coordinates[0], dep_vertiport_coordinates[1]
                list_of_possible_closest_edges = tools.find_list_of_closest_with_hash(x_dep, y_dep, hash_edges, min_x, min_y, x_step, y_step, resolution)
                dep_edge, dist_dep_edge = tools.find_closest_edge_in_list(x_dep, y_dep, list_of_possible_closest_edges, self.graph)

                x_arr, y_arr = arr_vertiport_coordinates[0], arr_vertiport_coordinates[1]
                list_of_possible_closest_edges = tools.find_list_of_closest_with_hash(x_arr, y_arr, hash_edges, min_x, min_y, x_step, y_step, resolution)
                arr_edge, dist_arr_edge = tools.find_closest_edge_in_list(x_arr, y_arr, list_of_possible_closest_edges, self.graph)

                dep = list(dep_edge)
                # print("before :", dep)
                arr = list(arr_edge)

                # If the departure is too far from the edge we store the dep or arr edge as departure/arrival_vertiport and
                # Set the value of is_departure_in_unconstrained
                if dist_arr_edge > self.protection_area:
                    surrounging_arr_polygon = self.find_surrounging_polygon(arr_vertiport_coordinates)
                    if surrounging_arr_polygon:
                        arr = surrounging_arr_polygon
                    arr_edge = arr_vertiport_coordinates
                    is_unconstrained_arrival = True
                else:
                    is_unconstrained_arrival = False
                if dist_dep_edge > self.protection_area:
                    surrounging_dep_polygon = self.find_surrounging_polygon(dep_vertiport_coordinates)
                    if surrounging_dep_polygon:
                        dep = surrounging_dep_polygon
                        # print("after :", dep)
                    dep_edge = dep_vertiport_coordinates
                    is_unconstrained_departure = True
                else:
                    is_unconstrained_departure = False

                dep_time = float(line[3][0:2])*3600 + float(line[3][3:5])*60 + float(line[3][6:8])
                flight_number = line[1]

                # Create drone object
                drone = dr.Drone(flight_number, dep, arr, dep_time, drone_model)
                drone.departure_vertiport = dep_vertiport_coordinates
                drone.arrival_vertiport = arr_vertiport_coordinates
                drone.deposit_time = deposit_time
                drone.is_unconstrained_departure = is_unconstrained_departure
                drone.is_unconstrained_arrival = is_unconstrained_arrival
                drone.dep_edge = dep_edge
                drone.arr_edge = arr_edge

                if len(line) > 9:
                    if line[9] != '':
                        drone.is_loitering_mission = True
                        drone.loitering_geofence = [float(_i) for _i in line[9:14]]

                # Check that the drone isn't already in the list
                drone_in_list = False
                for _d in self.droneList:
                    if _d.flight_number == drone.flight_number:
                        drone_in_list = True
                if not drone_in_list:
                    self.add_drone(drone)

    def find_conflicts(self):
        """Detect conflicts on the graph"""
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

    def find_conflicts_on_specified_drones(self, drones_to_check):
        """Detect conflicts on the graph"""
        conflict_list = []
        # drones_to_check = []
        # # Chaque drone n'est ajouté qu'une fois comme ça
        # for drone in self.droneList:
        #     if drone.flight_number in drones_to_check_flight_numbers:
        #         drones_to_check.append(drone)
        for i, drone in enumerate(drones_to_check):
            if i != len(drones_to_check)-1:
                for j in range(i+1, len(drones_to_check)):
                    # print(drone.flight_number, drones_to_check[j].flight_number)
                    # print(drone.path_object.path_dict, "\n", drones_to_check[j].path_object.path_dict)
                    t_edge = find_conflict_on_edges(drone, drones_to_check[j])
                    t_node = find_conflict_on_nodes(drone, drones_to_check[j], self.protection_area, self.graph_dual,
                                                    self.graph)
                    # print(t_edge, t_node)
                    if t_edge is not None:
                        if t_node is not None:
                            if t_edge <= t_node:
                                conflict_list.append((i, j, t_edge))
                            else:
                                conflict_list.append((i, j, t_node))
                    elif t_node is not None:
                        conflict_list.append((i, j, t_node))
        return conflict_list

    def find_surrounging_polygon(self, vertiport_coords):
        # Find the node closest to the vertiport :
        x, y = vertiport_coords[0], vertiport_coords[1]
        verti_point = shapely.geometry.Point(x, y)
        hash_nodes, hash_edges, min_x, min_y, x_step, y_step, resolution = self.hash_map
        nodes_list = tools.find_list_of_closest_with_hash(x,y,hash_nodes, min_x, min_y, x_step, y_step, resolution)
        start_node = tools.find_closest_node_in_list(x,y,nodes_list, self.graph)
        # print("STARTING NODE :", start_node)
        # Add the nodes neighbors until it loops AND creates a polygon that contains it
        open_list = [[start_node]]
        while len(open_list) > 0:
            current_step = open_list.pop(0)
            if len(current_step) > 9:
                # print("ON periphery")
                return None
            for neighbor_node in self.graph.neighbors(current_step[-1]):
                if len(current_step) > 3 and neighbor_node == current_step[0]:
                    new_step = current_step.copy()
                    new_step.append(neighbor_node)
                    pt_list = []
                    for node_idx in range(len(new_step)):
                        x1,y1 = self.graph.nodes[new_step[node_idx-1]]["x"], self.graph.nodes[new_step[node_idx-1]]["y"]
                        pt_list.append([x1, y1])
                    # print("CHECK POLYGON : ", new_step)
                    current_polygon = shapely.geometry.Polygon(pt_list)
                    # tools.scatter_graph(self.graph)
                    # plt.scatter(x, y, marker="o")
                    # plt.plot([pt[0] for pt in pt_list], [pt[1] for pt in pt_list])
                    # plt.show()
                    if current_polygon.contains(verti_point):
                        # print("CONTAINS")
                        return new_step
                    pass  # CHECK THAT POLYGON CONTAINS x,y VERTIPORT
                else:
                    new_step = current_step.copy()
                    if neighbor_node not in new_step:
                        new_step = current_step.copy()
                        new_step.append(neighbor_node)
                        open_list.append(new_step)
            open_list.sort(key=len)


def generate_scenarios(model, alts=None, turn_speeds = None):
    scenario_dict = dict()

    def get_dep_time(d):
        return d.dep_time
    sorted_drone_list = sorted(model.droneList, key=get_dep_time)
    for drone in sorted_drone_list:
        if drone.path_object is None:
            print("SKIPPED A DRONE : ", drone.flight_number)
            continue
        drone_id = drone.flight_number
        scenario_dict[drone.flight_number] = dict()
        # Here we need to add the first and last segment too
        lats, lons, turns = extract_lat_lon_turn_bool_from_path(drone, model)
        # scenario_dict[drone_id]['start_time'] = min(drone.path_object.path_dict.keys())
        scenario_dict[drone_id]['start_time'] = drone.dep_time
        # Add lats
        scenario_dict[drone_id]['lats'] = lats
        # Add lons
        scenario_dict[drone_id]['lons'] = lons
        # Add turnbool
        scenario_dict[drone_id]['turnbool'] = turns
        scenario_dict[drone_id]['turn_speeds'] = turn_speeds
        if alts is None:
            scenario_dict[drone_id]['alts'] = [25] * len(lats)
        else:
            scenario_dict[drone_id]['alts'] = alts[drone_id]
    return scenario_dict


def extract_lat_lon_turn_bool_from_path(drone, model):
    lats = []
    lon = []

    # Add the take off node:
    dep_vertiport = drone.departure_vertiport
    x_dep, y_dep = float(dep_vertiport[0]), float(dep_vertiport[1])
    lats.append(y_dep)
    lon.append(x_dep)
    # Add all the other nodes
    graph = model.graph
    for node in drone.path_object.path:
        node_x = graph.nodes[node]['x']
        node_y = graph.nodes[node]['y']
        lats.append(node_y)
        lon.append(node_x)
    # Add arrival node
    arr_vertiport = drone.arrival_vertiport
    x_arr, y_arr = float(arr_vertiport[0]), float(arr_vertiport[1])
    lats.append(y_arr)
    lon.append(x_arr)
    turn_bool = [False for _i in range(len(lats))]
    for i in range(1, len(turn_bool)-1):
        turn_bool[i] = turn_bool_function([lon[i-1], lats[i-1]], [lon[i], lats[i]], [lon[i+1], lats[i+1]])
    for i in range(0, len(turn_bool) - 2):
        # if len(drone.path_object.path[i-1]) == 6 and len(drone.path_object.path[i]) == 6 and len(drone.path_object.path[i+1]) == 6:
        if len(drone.path_object.path[i]) == 6:
            turn_bool[i] = False
    return lats, lon, turn_bool


def turn_bool_function(node1, node2, node3, turn_enabled=None):
    minimum_angle_to_apply_added_weight = 25
    x1, y1 = float(node1[0]), float(node1[1])
    x2, y2 = float(node2[0]), float(node2[1])
    x3, y3 = float(node3[0]), float(node3[1])
    angle = tools.angle_btw_vectors((x1, y1),(x2, y2),(x3, y3))
    if angle > minimum_angle_to_apply_added_weight:
        return True
    else:
        return False


def turn_cost_function(node1, node2, node3):
    """The function used to determine the cost to add to the edge in the dual graph to take into account the effect
    of turns on the drone speed and flight_time, it returns the 2 parts of the added cost of the turn (deceleration
    and acceleration) and the total added cost in time"""
    # TODO A changer en meme temps que le A*
    x1, y1 = float(node1["x"]), float(node1["y"])
    x2, y2 = float(node2["x"]), float(node2["y"])
    x3, y3 = float(node3["x"]), float(node3["y"])
    angle = tools.angle_btw_vectors((x1, y1), (x2, y2), (x3, y3))
    if angle > turn_angle_mini:
        return 0, 0, 0, True

    else:
        return 0, 0, 0, False


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


def check_conflict_on_node(t_drone1, t_drone2, d1_speed_after, d2_speed_after, protection_area):
    """Check for conflict on a node in the constrained airspace"""
    if t_drone1 >= t_drone2:
        if t_drone1 - t_drone2 < protection_area / d1_speed_after:
            return t_drone1
    if t_drone2 >= t_drone1:
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


def init_graph(graph):
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


def init_model(graph, graph_dual, drone_list_path, graph_hash, current_sim_time=None):
    """ initialise a model instance with a primal graph and a dual one and load drones from the specified time taking
    into account the current_time so the drones announced after this time aren't loaded."""
    model = Model(graph)
    model.hash_map = graph_hash
    model.droneList = []
    model.add_drones_from_file(drone_list_path, current_sim_time)
    model.set_graph_dual(graph_dual)
    return model


def init_graphs(graph_path, dual_path = None):
    raw_graph = nx.read_graphml(graph_path)
    graph = nx.Graph()
    # Creating a new graph without what's not needed
    for node in raw_graph.nodes:
        graph.add_node(node)
        graph.nodes[node]["x"] = float(raw_graph.nodes[node]["x"])
        graph.nodes[node]["y"] = float(raw_graph.nodes[node]["y"])
    for edge in raw_graph.edges:
        graph.add_edge(edge[0], edge[1])
        graph.edges[edge[0], edge[1]]["length"] = raw_graph.edges[edge]["length"]
        graph.edges[edge[0], edge[1]]["geometry"] = raw_graph.edges[edge]["geometry"]

    print("Removing nodes in static geofence")
    remove_nodes_in_geofences("graph_files/geo_data_new/crs_epsg_32633/geofences/geofences_big.gpkg", graph)

    if dual_path is None:
        print("CREATING DUAL GRAPH")
        graph_dual = dual_graph.create_dual(graph, turn_cost_function)

    else:
        _graph_dual = nx.read_graphml(dual_path)
        graph_dual = nx.DiGraph()
        for node in _graph_dual.nodes:
            new_node = node.split(",")
            new_node = (new_node[0].strip(" '("), new_node[1].strip(" ')"))
            # print(new_node)
            graph_dual.add_node(new_node)
            graph_dual.nodes[new_node]["x"] = _graph_dual.nodes[node]["x"]
            graph_dual.nodes[new_node]["y"] = _graph_dual.nodes[node]["y"]
            # print((new_node[0][3:-1], new_node[1][1:-3]))
            # print(new_node[0].strip(" '("), new_node[1].strip(" ')"))
        for edge in _graph_dual.edges:
            # print(edge)
            edge1 = tuple([node.strip("(),' ") for node in edge[0].split(",")])
            edge2 = tuple([node.strip("(),' ") for node in edge[1].split(",")])
            # print(edge1, edge2)
            new_edge = (edge1, edge2)
            # print(new_edge)
            graph_dual.add_edge(edge1, edge2)
            graph_dual.edges[new_edge]["is_turn"] = _graph_dual.edges[edge]["is_turn"]
            graph_dual.edges[new_edge]["length"] = _graph_dual.edges[edge]["length"]
            graph_dual.edges[new_edge]["total_turn_cost"] = _graph_dual.edges[edge]["total_turn_cost"]
            graph_dual.edges[new_edge]["post_turn_cost"] = _graph_dual.edges[edge]["post_turn_cost"]
            graph_dual.edges[new_edge]["pre_turn_cost"] = _graph_dual.edges[edge]["pre_turn_cost"]
            graph_dual.edges[new_edge]["angle"] = _graph_dual.edges[edge]["angle"]

    return graph, graph_dual


def remove_nodes_in_geofences(gpkg_file, graph):
    geofences_gpkg = geopandas.read_file(gpkg_file, crs= 'EPSG:32633')
    nodes_to_be_removed = []
    geofences_gpkg.to_crs(crs='EPSG:4326', inplace=True)
    for node in graph:
        pt = Point(graph.nodes[node]["x"], graph.nodes[node]["y"])
        for fence in geofences_gpkg.geometry:
            if fence.contains(pt):
                nodes_to_be_removed.append(node)
    for node in nodes_to_be_removed:
        graph.remove_node(node)



