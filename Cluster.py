import math
import random
import Astar2 as a2
import Path as pt
import main

# INTERVAL = 30
# DEPTH_CLUSTER = 5


class Cluster:
    def __init__(self, conflict_node, conflict_drones, graph, conflict_time, time_interval, cluster_depth):
        self.conflict_node = conflict_node
        self.conflictDrones = conflict_drones
        self.nodesList, self.edgesList = find_nodes_and_edges(conflict_node, graph, cluster_depth)
        self.drones = []
        self.obstacles = []
        self.conflict_time = conflict_time
        self.time_interval = time_interval
        self.cluster_depth = cluster_depth

    def find_drones(self, model, conflict_time):
        """Finds all the drones part of the given model that pass through the cluster
        between time t-INTERVALL and t+INTERVALL, with t a time given in seconds."""
        for drone in model.droneList:
            time_stamps = sorted(drone.path_object.path_dict.keys())
            for index, time in enumerate(time_stamps):
                if conflict_time - self.time_interval <= time <= conflict_time + self.time_interval:
                    node = drone.path_object.path_dict[time]
                    node_minus_1, node_plus_1 = None, None
                    edge_before, edge_after = None, None
                    if index > 0:
                        node_minus_1 = drone.path_object.path_dict[time_stamps[index-1]]
                        edge_before = (node_minus_1, node)
                    if index+1 < len(time_stamps)-1:
                        node_plus_1 = drone.path_object.path_dict[time_stamps[index-1]]
                        edge_after = (node, node_plus_1)
                    # if node in self.nodesList:
                    #     self.drones.append(drone)
                    if edge_after in self.edgesList or edge_before in self.edgesList:
                        self.drones.append(drone)
                        break

    def solve_cluster_dual(self, model, max_permutations):
        # RQ Si on ne resoud pas le cluster on peut avoir plus de conflits a la fin qu'au debut ...
        """Solve the cluster by solving the problem sequentially over all possible permutations or at least a certain
        number"""
        # Contain all possible permutations of drones order
        possible_permutations_nb = math.factorial(len(self.drones))
        # print("number of permutations ", possible_permutations_nb)
        best_time = 10000000

        # Compute the new best path for the drone taking into account the other drones paths as constraints
        def new_path_dual(current_drone, constraint_primal_dict):
            """Find the new shortest path for the current drone, taking into account the constraints added
            by the already previously computed drones
            constraint_primal_dict = {dual_node : [arrival_time, next_node, next_node_time, drone]}
            If the drone actually juste pass by the first node but doesn't exit at the ending one, only the arrival time
            is filled"""
            # found = False
            # i = 0
            # # Searching for the current node to be set as departure node
            # while not found:
            #     if current_drone.path_object.path[i] in self.nodesList:
            #         departure_node = current_drone.path_object.path[i]
            #         found = True
            #     else:
            #         i += 1
            #
            time_stamps = sorted(current_drone.path_object.path_dict.keys())
            for index, time in enumerate(time_stamps):
                if time > self.conflict_time - self.time_interval:
                    i = index
                    departure_time = time
                    departure_node = current_drone.path_object.path_dict[time]
                    # print(time, self.conflict_time)
                    break

            # Find the departure node in the cluster and the time interval
            # Find first node in the cluster
            # move forward to be as close to interval as possible
            # d_time_stamps = sorted(current_drone.path_object.path_dict.keys())
            # for t_stamps_index, time in enumerate(d_time_stamps):
            #     node = current_drone.path_object.path_dict[time]
            #     if node in self.nodesList:
            #         departure_node = node
            #         departure_time = time
            #         if time > self.conflict_time - self.time_interval:
            #             break
            #         if t_stamps_index + 1 < len(d_time_stamps)-1:
            #             if d_time_stamps[t_stamps_index + 1] == self.conflict:
            #                 break
            #         else:
            #             break
            #
            # for time in current_drone.path_object.path_dict:
            #     if self.conflict_time - self.time_interval <= time or self.conflict_time + self.time_interval:
            #         if current_drone.path_object.path_dict[time] in self.nodesList:
            #             departure_node = current_drone.path_object.path_dict[time]
            #             # print("dep ", departure_node)
            #             # print("conflit ", self.conflict)
            #             # if self.conflict in current_drone.path_object.path:
            #             #     print("dep index ", current_drone.path_object.path.index(departure_node), "conf index ", current_drone.path_object.path.index(self.conflict))
            #             break
            #
            # i = list(current_drone.path_object.path_dict.values()).index(departure_node)
            # departure_time = list(current_drone.path_object.path_dict.keys())[i]

            # Create the dual departure and arrival for the A* dual algorithm
            dep_dual = ("S" + departure_node, departure_node)
            arr_dual = (drone.arr, drone.arr + "T")
            # Computing shortest path from the first node in the cluster to the arrival node
            path_solution = a2.astar_dual(model, dep_dual, arr_dual, drone, departure_time, constraint_primal_dict)
            new_solution = None
            # If a solution has been found
            if path_solution is not None:
                # Checking if there is a previous path already saved
                # if not current_drone.path_object.previous_path == []:
                #     previous_path = current_drone.path_object.previous_path.path
                # else:
                #     previous_path = []
                # The path is modified starting from the first waypoint taken into account in the cluster
                new_path = current_drone.path_object.path[:i].copy() + path_solution.path.copy()
                # Checking that the new path is not the same as the last one or the current one
                if current_drone.path_object.path != new_path :
                    new_solution = pt.Path(current_drone.dep_time, new_path.copy())
                    # new_solution.previous_path = current_drone.path_object
                    new_solution.set_path(new_path.copy(), model.graph, model.graph_dual, current_drone)
                    new_solution.discretize_path(5, model.graph, current_drone)
                    new_solution.flight_time_and_distance(model.graph, current_drone)
            # Adding the constraints, only change anything if the path has changed
            constraint_primal_dict = add_constraints_dual(constraint_primal_dict, current_drone, new_solution)
            # print(constraint_dict)
            return constraint_primal_dict, new_solution

        # Looking for the best order to solve the drones in (this allows to find a more efficient solution and avoids
        # some cases where one drone could be completely out of solution because of the starting conditions)

        # print([d.flight_number for d in self.drones])
        drones_list = self.drones.copy()
        random.shuffle(drones_list)

        previous_path_dict = dict()
        for drone in self.drones:
            previous_path_dict[drone.flight_number] = drone.path_object

        solution_path_dict = dict()
        # Initialize the constraint dict
        if model.initial_constraints_dict is not None:
            constraint_dict = model.initial_constraints_dict
        else:
            constraint_dict = dict()
        # Add all the constraint of the flights that aren't included in the cluster up to conflict_time +1 node
        for drone_to_add in model.droneList:
            # Only add the drones that aren't in the cluster
            if drone_to_add.flight_number not in [d.flight_number for d in self.drones]:
                # Only add the drones path up to one node after the conflict_time
                constraint_dict = add_initial_constraints_dual(constraint_dict, drone_to_add, drone_to_add.path_object, self.conflict_time + self.time_interval)

        # Computing the best path for each drone sequentially (each drone take the already computed ones as constraints)
        for drone in drones_list:
            constraint_dict, path = new_path_dual(drone, constraint_dict)
            # Separate the changed and unchanged path
            if path is not None:
                drone.path_object = path
            solution_path_dict[drone.flight_number] = path
        for flight_number in solution_path_dict:
            if solution_path_dict[flight_number] is None:
                print("No conflict free solution found")
                for drone in self.drones:
                    drone.path_object = previous_path_dict[drone.flight_number]
                break


def add_constraints_dual(constraint_dict, drone, path):
    """Take the path of drone and turn it into constraints to add to the constraint_dict of the primal graph
    constraint format is { node : [[time_at_node, next_node, time_at_next_node, drone], [...]] }
    The constraint_dict is modified in here as it is mutable"""
    # If path is None, this means that the drone path hasn't been modified, so we use the drone.path_object.path as
    # the path to create the constraints.
    if path is None:
        # print("PATH IS NONE")
        path_to_use = drone.path_object
    else:
        # print("NEW PATH USED")
        path_to_use = path
    time_list = list(path_to_use.path_dict)
    for index, time in enumerate(time_list):
        node = path_to_use.path_dict[time]
        # Check if the node is actually the arrival node, if it is the case next_node = node
        if index + 1 < len(time_list):
            next_time = time_list[index + 1]
            next_node = path_to_use.path_dict[next_time]
            constraint = [time, next_node, next_time, drone]
        else:
            constraint = [time, node, time, drone]
        if node in constraint_dict:
            constraint_dict[node].append(constraint)
        else:
            constraint_dict[node] = [constraint]
    # print("DRONE PATH",drone.flight_number, "ADDED TO CONSTRAINT ",path_to_use.path_dict)
    return constraint_dict


def add_initial_constraints_dual(constraint_dict, drone, path_object, conflict_time):
    """Take the path of drone and turn it into constraints to add to the constraint_dict of the primal graph
    constraint format is { node : [[time_at_node, next_node, time_at_next_node, drone], [...]] }
    The constraint_dict is modified in here as it is mutable"""
    # If path is None, this means that the drone path hasn't been modified, so we use the drone.path_object.path as
    # the path to create the constraints.
    time_list = list(path_object.path_dict)
    for index, time in enumerate(time_list):
        node = path_object.path_dict[time]
        # Check if the node is actually the arrival node, if it is the case next_node = node
        if index + 1 < len(time_list):
            next_time = time_list[index + 1]
            next_node = path_object.path_dict[next_time]
            constraint = [time, next_node, next_time, drone]
        else:
            constraint = [time, node, time, drone]
        if node in constraint_dict:
            constraint_dict[node].append(constraint)
        else:
            constraint_dict[node] = [constraint]
        if time > conflict_time:
            break
    # print("DRONE PATH",drone.flight_number, "ADDED TO CONSTRAINT ",path_to_use.path_dict)
    return constraint_dict


def get_time(d, cluster):
    found = False
    i = 0
    while not found:
        if d.path_object.path_object[i] in cluster.nodesList:
            dep = d.path_object.path_object[i]
            found = True
            return list(d.path_dict.keys())[list(d.path_object.path_dict.values()).index(dep)]
        else: 
            i += 1


def find_nodes_and_edges(conflict_node, graph, cluster_depth):
    """Finds all the nodes and edges present in the cluster, which starts at the given
    conflict node (str). Also defines the size of the cluster with the first "for" loop (Depth).
    Sets the color of the edges and nodes within the cluster to represent them on the graphical map."""
    nodes_list = []
    nodes_queue = [conflict_node]

    # Search recursively for the linked nodes up to a distance of DEPTH
    def find_nodes(node_queue, list_nodes, depth):
        if depth == cluster_depth:
            return list_nodes
        new_node_queue = []
        for node in node_queue:
            graph.nodes[node]['color'] = 'yellow'
            list_nodes.append(node)
            for n in graph.adj[node]:
                if n not in list_nodes:
                    new_node_queue.append(n)
        return find_nodes(new_node_queue.copy(), list_nodes, depth + 1)

    nodes_list = find_nodes(nodes_queue, nodes_list, 0)
    edges_list = []
    for edge in graph.edges:
        if edge[0] in nodes_list and edge[1] in nodes_list:
            edges_list.append(edge)
            graph.edges[edge]['color'] = 'yellow'

    return nodes_list, edges_list

#
# def solve_cluster_dual_with_permutations(self, model, max_permutations):
#     # RQ Si on ne resoud pas le cluster on peut avoir plus de conflits a la fin qu'au debut ...
#     """Solve the cluster by solving the problem sequentially over all possible permutations or at least a certain
#     number"""
#     # Contain all possible permutations of drones order
#     possible_permutations_nb = math.factorial(len(self.drones))
#     # print("number of permutations ", possible_permutations_nb)
#     best_time = 10000000
#
#     # Compute the new best path for the drone taking into account the other drones paths as constraints
#     def new_path_dual(current_drone, constraint_primal_dict):
#         """Find the new shortest path for the current drone, taking into account the constraints added
#         by the already previously computed drones
#         constraint_primal_dict = {dual_node : [arrival_time, next_node, next_node_time, drone]}
#         If the drone actually juste pass by the first node but doesn't exit at the ending one, only the arrival time
#         is filled"""
#         found = False
#         i = 0
#         # Searching for the current node to be set as departure node
#         while not found:
#             if current_drone.path_object.path[i] in self.nodesList:
#                 departure_node = current_drone.path_object.path[i]
#                 found = True
#             else:
#                 i += 1
#
#         # Find the departure node in the cluster and the time interval
#         # Find first node in the cluster
#         # move forward to be as close to interval as possible
#         # d_time_stamps = sorted(current_drone.path_object.path_dict.keys())
#         # for t_stamps_index, time in enumerate(d_time_stamps):
#         #     node = current_drone.path_object.path_dict[time]
#         #     if node in self.nodesList:
#         #         departure_node = node
#         #         departure_time = time
#         #         if time > self.conflict_time - self.time_interval:
#         #             break
#         #         if t_stamps_index + 1 < len(d_time_stamps)-1:
#         #             if d_time_stamps[t_stamps_index + 1] == self.conflict:
#         #                 break
#         #         else:
#         #             break
#         #
#         # for time in current_drone.path_object.path_dict:
#         #     if self.conflict_time - self.time_interval <= time or self.conflict_time + self.time_interval:
#         #         if current_drone.path_object.path_dict[time] in self.nodesList:
#         #             departure_node = current_drone.path_object.path_dict[time]
#         #             # print("dep ", departure_node)
#         #             # print("conflit ", self.conflict)
#         #             # if self.conflict in current_drone.path_object.path:
#         #             #     print("dep index ", current_drone.path_object.path.index(departure_node), "conf index ", current_drone.path_object.path.index(self.conflict))
#         #             break
#
#         i = list(current_drone.path_object.path_dict.values()).index(departure_node)
#         departure_time = list(current_drone.path_object.path_dict.keys())[i]
#
#         # Create the dual departure and arrival for the A* dual algorithm
#         dep_dual = ("S" + departure_node, departure_node)
#         arr_dual = (drone.arr, drone.arr + "T")
#         # Computing shortest path from the first node in the cluster to the arrival node
#         path_solution = a2.astar_dual(model, dep_dual, arr_dual, drone, departure_time, constraint_primal_dict)
#         new_solution = None
#         # If a solution has been found
#         if path_solution is not None:
#             # Checking if there is a previous path already saved
#             if not current_drone.path_object.previous_path == []:
#                 previous_path = current_drone.path_object.previous_path.path
#             else:
#                 previous_path = []
#             # The path is modified starting from the first waypoint taken into account in the cluster
#             new_path = current_drone.path_object.path[:i].copy() + path_solution.path.copy()
#             # Checking that the new path is not the same as the last one or the current one
#             if current_drone.path_object.path != new_path and new_path != previous_path:
#                 new_solution = pt.Path(current_drone.dep_time, new_path.copy())
#                 new_solution.previous_path = current_drone.path_object
#                 new_solution.set_path(new_path.copy(), model.graph, model.graph_dual, current_drone)
#                 new_solution.discretize_path(5, model.graph, current_drone)
#                 new_solution.flight_time_and_distance(model.graph, current_drone)
#         # Adding the constraints, only change anything if the path has changed
#         constraint_primal_dict = add_constraints_dual(constraint_primal_dict, current_drone, new_solution)
#         # print(constraint_dict)
#         return constraint_primal_dict, new_solution
#
#     # Looking for the best order to solve the drones in (this allows to find a more efficient solution and avoids
#     # some cases where one drone could be completely out of solution because of the starting conditions)
#     permutation_count = 0
#     #TODO retirer ou faire les permut
#     while permutation_count < min(possible_permutations_nb, max_permutations):
#         # print([d.flight_number for d in self.drones])
#         drones_list = self.drones.copy()
#         random.shuffle(drones_list)
#         permutation_count += 1
#
#         # Initialize the constraint dict
#         if model.initial_constraints_dict is not None:
#             constraint_dict = model.initial_constraints_dict
#         else:
#             constraint_dict = dict()
#         # Add all the constraint of the flights that aren't included in the cluster up to conflict_time +1 node
#         for drone_to_add in model.droneList:
#             # Only add the drones that aren't in the cluster
#             if drone_to_add.flight_number not in [d.flight_number for d in self.drones]:
#                 # Only add the drones path up to one node after the conflict_time
#                 constraint_dict = add_initial_constraints_dual(constraint_dict, drone_to_add, drone_to_add.path_object, self.conflict_time)
#
#         paths_changed = []
#         paths_unchanged = []
#         # Computing the best path for each drone sequentially (each drone take the already computed ones as
#         # constraints)
#         for drone in drones_list:
#             constraint_dict, path = new_path_dual(drone, constraint_dict)
#             # Separate the changed and unchanged path
#             if path is not None:
#                 paths_changed.append(path)
#             else:
#                 paths_unchanged.append(drone.path_object)
#         total_time = 0
#         # Compute the total flight time using this permutation
#         if len(paths_changed) != 0:
#             for p in paths_changed:
#                 total_time += p.flightTime
#             for p in paths_unchanged:
#                 total_time += p.flightTime
#         else:
#             total_time = best_time + 10
#         # Check if the current permutation is better than the current best one
#         if total_time <= best_time:
#             best_time = total_time
#             best_permutation = drones_list
#         else:
#             best_permutation = None
#             print("No solutions found")
#             # for d in drones_list:
#             #     main.draw_solution_drone(d, model.graph, "conflicts/plt_sol_{}.png".format(d.flight_number))
#
#
#     # After iterating over the permutations we use the best found permutation and do the process
#     # one last time to save the best paths found
#     if best_permutation is not None:
#         if model.initial_constraints_dict is not None:
#             constraint_dict = model.initial_constraints_dict
#         else:
#             constraint_dict = dict()
#         # Add all the constraint of the flights that aren't included in the cluster up to conflict_time +1 node
#         for drone_to_add in model.droneList:
#             # Only add the drones that aren't in the cluster
#             if drone_to_add.flight_number not in [d.flight_number for d in self.drones]:
#                 # Only add the drones path up to one node after the conflict_time
#                 constraint_dict = add_initial_constraints_dual(constraint_dict, drone_to_add, drone_to_add.path_object, self.conflict_time)
#         for drone in best_permutation:
#             constraint_dict, path = new_path_dual(drone, constraint_dict)
#             if path is not None:
#                 drone.path_object = path
#     # else:
#         # print("No solutions found")
#         # for d in self.drones:
#         #     print("Flight number", d.flight_number)
#         #     print("Path", d.path_object.path_dict)
#         # print("constraints", constraint_dict)
