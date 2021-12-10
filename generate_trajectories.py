"""Generate trajectories to be used in the optim model"""
import Model
import dual_graph
import networkx
import Model as md
import math
from main import turn_cost_function
import matplotlib.pyplot as plt
import Astar2 as a2
import Path
import numpy as np
import itertools


graph_file_path = "graph_files/processed_graphM2.graphml"
# graph_file_path = "graph_files/geo_data/crs_epsg_32633/road_network/crs_4326_cleaned_simplified_network/cleaned_simplified.graphml"
drone_list_file_path = 'graph_files/drones.txt'
protection_area = 30


def main():
    ####
    # Init
    graph, graph_dual = init_graphs(graph_file_path)
    model = init_model(graph, graph_dual, drone_list_file_path, protection_area)
    drone_trajectories_dict = dict()

    ####
    # Generate
    multiple_point_bool = True
    if multiple_point_bool:
        for drone in model.droneList[0:2]:
            print(drone.flight_number)
            points_to_explore_from_shortest = generate_points_from_shortest_path(model, drone, graph, 2, 3)
            drone_trajectories_dict[drone.flight_number] = generate_multiple_point_trajectories(drone, graph_file_path, points_to_explore_from_shortest, model)

    one_point_bool = False
    if one_point_bool:
        points_to_explore_for_one = generate_points_to_pass_by_for_one_point(graph)
        min_number_of_trajectories = 100
        for drone in model.droneList:
            drone_trajectories_dict[drone.flight_number] = generate_one_point_trajectories(drone, graph_file_path, points_to_explore_for_one, model)
            min_number_of_trajectories = min(min_number_of_trajectories, len(drone_trajectories_dict[drone.flight_number]))

    display_traj = False
    if display_traj:
        for traj in drone_trajectories_dict["D1"]:
            for node in graph.nodes:
                plt.scatter(graph.nodes[node]["x"], graph.nodes[node]["y"], color='blue')
            x = [graph.nodes[node]["x"] for node in traj]
            y = [graph.nodes[node]["y"] for node in traj]
            plt.plot(x, y, marker='*', color='red')
            plt.show()

    # Keep only certain number of traj for each drone and add an id
    new_dict = dict()
    number_of_traj_to_keep = 10
    id_trajectory = 0
    for drone_flight_number in drone_trajectories_dict:
        new_dict[drone_flight_number] = dict()
        for i in range(number_of_traj_to_keep):
            new_dict[drone_flight_number][id_trajectory] = [drone_trajectories_dict[drone_flight_number][i]]
            id_trajectory += 1
    drone_trajectories_dict = new_dict
    # print(drone_trajectories_dict)

    ####
    # Metrics (length, compatibility)

    # For each trajectory determine its total travel time and add the path_object
    for drone_flight_number in drone_trajectories_dict:
        for id_trajectory in drone_trajectories_dict[drone_flight_number]:
            current_drone = None
            for drone in model.droneList:
                if drone.flight_number == drone_flight_number:
                    current_drone = drone
                    break
            path = Path.Path(drone.dep_time, [])
            path.set_path(drone_trajectories_dict[drone_flight_number][id_trajectory][0], graph, graph_dual, drone)
            total_time = max(list(path.path_dict.keys()))
            drone_trajectories_dict[drone_flight_number][id_trajectory].append(total_time)
            drone_trajectories_dict[drone_flight_number][id_trajectory].append(path)

    display_traj2 = False
    if display_traj2:
        for drone_flight_number in drone_trajectories_dict:
            # print(drone_flight_number)
            for id_trajectory in drone_trajectories_dict[drone_flight_number]:
                traj = drone_trajectories_dict[drone_flight_number][id_trajectory][0]
                # print(drone_trajectories_dict[drone_flight_number][id_trajectory][1])
                for node in graph.nodes:
                    plt.scatter(graph.nodes[node]["x"], graph.nodes[node]["y"], color='blue')
                x = [graph.nodes[node]["x"] for node in traj]
                y = [graph.nodes[node]["y"] for node in traj]
                plt.plot(x, y, marker='*', color='red')
                plt.show()
    display_best_traj = False
    if display_best_traj:
        for drone_flight_number in drone_trajectories_dict:
            id = list(drone_trajectories_dict[drone_flight_number].keys())[0]
            traj = drone_trajectories_dict[drone_flight_number][id][0]
            for node in graph.nodes:
                plt.scatter(graph.nodes[node]["x"], graph.nodes[node]["y"], color='blue')
            x = [graph.nodes[node]["x"] for node in traj]
            y = [graph.nodes[node]["y"] for node in traj]
            plt.plot(x, y, marker='*', color='red')
            plt.show()

    # For each trajectory determine if there is a conflict with another
    size = 0
    for drone_flight_number in drone_trajectories_dict:
        size += len(drone_trajectories_dict[drone_flight_number])
    # print(size)
    compatibility_matrix = np.zeros((size, size))
    # 1 if flight i and j are compatible, 0 if there is a conflict
    for i in range(size):
        for j in range(size):
            flight_number1 = "D" + str(1 + (i // number_of_traj_to_keep))
            flight_number2 = "D" + str(1 + (j // number_of_traj_to_keep))
            # print(i, j)
            # print(flight_number1, flight_number2)
            # print("id", i, j)
            # print("fn", flight_number1, flight_number2)
            drone1 = return_drone_from_flight_number(model, flight_number1)
            drone1.path = drone_trajectories_dict[flight_number1][i][2]
            drone2 = return_drone_from_flight_number(model, flight_number2)
            drone2.path = drone_trajectories_dict[flight_number2][j][2]
            # print(drone1.path.path_dict)
            # print(return_drone_from_flight_number(model, "D1").path.path_dict)
            # print(drone2.path.path_dict)

            if model.find_conflicts_on_specified_drones([drone1.flight_number, drone2.flight_number]) == []:
                print("YON")
            else:
                print("YAYAYA")


def return_drone_from_flight_number(model, flight_number):
    for drone in model.droneList:
        if drone.flight_number == flight_number:
            return drone
    else:
        print("Drone not in list")
        raise Exception


def check_trajectories_are_compatible(traj1, traj2, drone1, drone2):
    pass


def compute_travel_time(traj, drone):
    pass


def generate_one_point_trajectories(drone, graph_path, points_to_explore, model):
    trajectories = []
    graph, graph_dual = init_graphs(graph_path)
    drone_dep_dual = ("S" + drone.dep, drone.dep)
    drone_arr_dual = (drone.arr, drone.arr + "T")
    shortest_path = a2.astar_dual(model, drone_dep_dual, drone_arr_dual, drone, drone.dep_time)
    trajectories.append(shortest_path.path)
    for point in points_to_explore:
        intermediary_dep_dual = ("S" + point, point)
        intermediary_arr_dual = (point, point + "T")
        first_part = a2.astar_dual(model, drone_dep_dual, intermediary_arr_dual, drone, drone.dep_time).path
        second_part = a2.astar_dual(model, intermediary_dep_dual, drone_arr_dual, drone, drone.dep_time).path
        if first_part is None or second_part is None:
            continue
        trajectory = first_part + second_part[1:]
        if len(trajectory) == len(set(trajectory)):
            trajectories.append(trajectory)
        elif len(trajectory) == len(set(trajectory)) + 1:
            # Means the drone does a u-turn at the intermediary point, if so removes it and the duplicated node
            trajectory.pop(len(first_part)-1)
            trajectory.pop(len(first_part)-1)
            if not check_traj_is_possible(graph, trajectory):
                if trajectory not in trajectories:
                    trajectories.append(trajectory)
    return trajectories


def generate_multiple_point_trajectories(drone, graph_path, list_of_points_to_explore, model):
    trajectories = []
    graph, graph_dual = init_graphs(graph_path)
    drone_dep_dual = ("S" + drone.dep, drone.dep)
    drone_arr_dual = (drone.arr, drone.arr + "T")
    shortest_path = a2.astar_dual(model, drone_dep_dual, drone_arr_dual, drone, drone.dep_time)
    trajectories.append(shortest_path.path)
    waypoints_list = [[drone.dep]]
    index = 0
    while index != len(list_of_points_to_explore):
        new_temp_traj = []
        for node in list_of_points_to_explore[index]:
            for waypoints in waypoints_list:
                new_temp_traj.append(waypoints + [node])
        waypoints_list = new_temp_traj
        index += 1
    for waypoints in waypoints_list:
        waypoints.append(drone.arr)
    for waypoints in waypoints_list:
        traj = []
        path_is_none = False
        for waypoint_index in range(len(waypoints) - 1):
            dep_dual = ("S" + waypoints[waypoint_index], waypoints[waypoint_index])
            arr_dual = (waypoints[waypoint_index+1], waypoints[waypoint_index+1] + "T")
            path = a2.astar_dual(model, dep_dual, arr_dual, drone, drone.dep_time).path
            if path is None:
                path_is_none = True
            traj += path
        if not path_is_none:
            # print(traj)
            index = 0
            while index < len(traj) - 2:
                index += 1
                if traj[index] == traj[index + 1]:
                    traj.pop(index)
            if check_traj_is_possible(graph, traj):
                if len(traj) == len(list(set(traj))):
                    trajectories.append(traj)
    # print(len(trajectories))
    for index in range(len(trajectories)-1, 0, -1):
        if trajectories[index] in trajectories[0:index]:
            trajectories.pop(index)
    # print(len(trajectories))
    return trajectories


def generate_points_from_shortest_path(model, drone, graph, steps, depth):
    # Create 2 nodes combinaisons from the shortest_path nodes' succesors
    trajectories = []
    # 1 A* to find the shortest path
    drone_dep_dual = ("S" + drone.dep, drone.dep)
    drone_arr_dual = (drone.arr, drone.arr + "T")
    shortest_path = a2.astar_dual(model, drone_dep_dual, drone_arr_dual, drone, drone.dep_time).path
    index_list = [(i+1)*(len(shortest_path)//(steps+1)) for i in range(steps)]
    nodes_list=[]
    for index in index_list:
        current_node_list = []
        temp_node_list = []
        initial_node = shortest_path[index]
        temp_node_list.append(initial_node)
        current_node_list.append(initial_node)
        for i in range(depth):
            for node in current_node_list:
                temp_node_list = temp_node_list + list(graph.neighbors(node))
            current_node_list = current_node_list + temp_node_list
            current_node_list = list(set(current_node_list))
        nodes_list.append(current_node_list)
    return nodes_list



def generate_points_to_pass_by_for_one_point(graph):
    # Quadriller l'espace
    # Find max x,y et min x,y
    steps = 10
    # nodes_to_explore = [[None for i in range(steps)] for j in range(steps)]
    nodes_to_explore = []
    max_x = -math.inf
    max_y = -math.inf
    min_x = math.inf
    min_y = math.inf
    for node in graph.nodes:
        max_x = max(max_x, graph.nodes[node]["x"])
        min_x = min(min_x, graph.nodes[node]["x"])
        max_y = max(max_y, graph.nodes[node]["y"])
        min_y = min(min_y, graph.nodes[node]["y"])
    x_range = [min_x + i*(max_x-min_x)/(steps - 1) for i in range(steps)]
    y_range = [min_y + i*(max_y-min_y)/(steps - 1) for i in range(steps)]
    x_y_pos = [[(x, y) for y in y_range] for x in x_range]
    for x in range(len(x_y_pos)):
        for y in range(len(x_y_pos[0])):
            best_dist = 5000000
            closest_node = None
            for node in graph.nodes:
                dist = math.sqrt((x_y_pos[x][y][0] - graph.nodes[node]["x"])**2 + (x_y_pos[x][y][1] - graph.nodes[node]["y"])**2)
                if dist < best_dist:
                    closest_node = node
                    best_dist = dist
            # nodes_to_explore[x][y] = closest_node
            nodes_to_explore.append(closest_node)
    return set(nodes_to_explore)


def check_traj_is_possible(graph, traj):
    for index in range(len(traj)-1):
        if (traj[index], traj[index+1]) not in graph.edges and (traj[index+1], traj[index]) not in graph.edges:
            return False
    return True

def init_graphs(graph_path):
    raw_graph = networkx.read_graphml(graph_path)
    graph = networkx.Graph()
    # Creating a new graph without what's not needed
    for node in raw_graph.nodes:
        graph.add_node(node)
        graph.nodes[node]["x"] = float(raw_graph.nodes[node]["x"])
        graph.nodes[node]["y"] = float(raw_graph.nodes[node]["y"])
    for edge in raw_graph.edges:
        graph.add_edge(edge[0], edge[1])
        graph.edges[edge[0], edge[1]]["length"] = raw_graph.edges[edge]["length"]
        graph.edges[edge[0], edge[1]]["geometry"] = raw_graph.edges[edge]["geometry"]
    graph_dual = dual_graph.create_dual(graph, turn_cost_function)
    return graph, graph_dual


def init_model(graph, graph_dual, drone_list_path, protection_area_size, current_sim_time=None):
    """ initialise a model instance with a primal graph and a dual one and load drones from the specified time taking
    into account the current_time so the drones announced after this time aren't loaded."""
    model = md.Model(graph, protection_area_size)
    model.droneList = []
    if current_sim_time is not None:
        model.add_drones_from_file(drone_list_path, current_sim_time)
    else:
        model.add_drones_from_file_without_deposit_time(drone_list_path)
    model.set_graph_dual(graph_dual)
    return model


main()