import math
import random
import networkx

import Astar2 as a2
import itertools
import Path as pt

INTERVAL = 30
DEPTH_CLUSTER = 3


class Cluster:
    def __init__(self, conflictNode, conflictDrones, graph,
                 initial_constraint_nodes=None, initial_constraint_edges=None):
        self.conflict = conflictNode
        self.conflictDrones = conflictDrones
        self.nodesList, self.edgesList = self.find_nodes_and_edges(conflictNode, graph)
        self.drones = []
        self.obstacles = []
        self.initial_constraint_nodes = initial_constraint_nodes
        if initial_constraint_nodes is None:
            self.initial_constraint_nodes = {}
        self.initial_constraint_edges = initial_constraint_edges
        if initial_constraint_edges is None:
            self.initial_constraint_edges = {}

    def find_nodes_and_edges(self, conflictNode, graph):
        """Finds all the nodes and edges present in the cluster, which starts at the given
        conflict node (str). Also defines the size of the cluster with the first "for" loop (Depth).
        Sets the color of the edges and nodes within the cluster to represent them on the graphical map."""
        nodes_list = []
        nodes_queue = [conflictNode]

        # Search recursively for the linked nodes up to a distance of DEPTH
        def find_nodes(node_queue, list_nodes, graph, depth):
            if depth == DEPTH_CLUSTER:
                return list_nodes
            new_node_queue = []
            for node in node_queue:
                graph.nodes[node]['color'] = 'yellow'
                list_nodes.append(node)
                for n in graph.adj[node]: 
                    if n not in list_nodes:
                        new_node_queue.append(n)
            return find_nodes(new_node_queue.copy(), list_nodes, graph, depth + 1)
            
        nodes_list = find_nodes(nodes_queue, nodes_list, graph, 0)
        edges_list = []
        for edge in graph.edges:
            if edge[0] in nodes_list and edge[1] in nodes_list:
                edges_list.append(edge)
                graph.edges[edge]['color'] = 'yellow'
        
        return nodes_list, edges_list

    # TODO Definition de la zone du cluster pour le moment juste taille = 3?
    def find_drones(self, model, t):
        """Finds all the drones part of the given model that pass through the cluster
        between time t-INTERVALL and t+INTERVALL, with t a time given in seconds."""
        for drone in model.droneList:
            for h in drone.path_object.path_dict:
                if t-INTERVAL <= h <= t+INTERVAL:
                    dronePath = drone.path_object
                    node = dronePath.path_dict[h]
                    i = dronePath.path.index(node)
                    # TODO C'est quoi l'exception la ?
                    try:
                        if (dronePath.path[i], dronePath.path[i + 1]) in self.edgesList or (dronePath.path[i + 1], dronePath.path[i]) in self.edgesList:
                            self.drones.append(drone)
                            break
                        elif (dronePath.path[i - 1], dronePath.path[i]) in self.edgesList or (dronePath.path[i], dronePath.path[i - 1]) in self.edgesList:
                            self.drones.append(drone)
                            break
                    except Exception:
                        if (dronePath.path[i - 1], dronePath.path[i]) in self.edgesList or (dronePath.path[i], dronePath.path[i - 1]) in self.edgesList:
                            self.drones.append(drone)
                            break   

    # def solve_cluster_delay(self, model):
    #     perms = list(itertools.permutations(self.drones))
    #     best_time = 10000000
    #
    #     def new_path(drone, constraints_nodes, constraints_edges):
    #         found = False
    #         i = 0
    #         drone_path = drone.path
    #         while not found:
    #             if drone_path.path[i] in self.nodesList:
    #                 dep = drone_path.path[i]
    #                 found = True
    #             else:
    #                 i += 1
    #         pathSol = None
    #         iDict = list(drone_path.pathDict.values()).index(dep)
    #         timeDep = list(drone_path.pathDict.keys())[iDict]
    #         delay = -5
    #         while pathSol == None and delay < 60:
    #             delay += 5
    #             pathSol = a2.Astar(model.graph, dep, drone.arr, drone, timeDep + delay, constraints_nodes, constraints_edges)
    #
    #         newSol = None
    #
    #         newPath = drone.path.path[:i].copy() + pathSol.path.copy()
    #         try:
    #             pDelay = drone.path.delay[dep]
    #         except:
    #             pDelay = 0
    #         try:
    #             ppDelay = drone.path.previousPath.delay[dep]
    #         except:
    #             ppDelay = 0
    #         try:
    #             previousPath = drone.path.previousPath.path
    #         except:
    #             previousPath = []
    #
    #         if (drone.path.path != newPath or (drone.path.path == newPath and pDelay != delay)) and (newPath != previousPath or (newPath == previousPath and ppDelay != delay)):
    #             try:
    #                 hDep = drone.path.hStart
    #             except:
    #                 hDep = drone.hDep
    #             newSol = pt.Path(hDep, newPath.copy())
    #             newSol.delay = drone.path.delay.copy()
    #             for node in newSol.delay.copy():
    #                 if node not in newPath:
    #                     newSol.delay.pop(node)
    #             if delay != 0:
    #                 try:
    #                     newSol.delay[dep] += delay
    #                 except KeyError:
    #                     newSol.delay[dep] = delay
    #             if dep == drone.dep and delay != 0:
    #                 newSol.hStart += delay
    #                 newSol.delay.pop(dep)
    #             newSol.previousPath = drone.path
    #             newSol.add_path(newPath.copy(), model.graph, drone)
    #             newSol.discretize_path(5, model.graph, drone)
    #             newSol.flight_time_and_distance(model.graph, drone)
    #             newSol.flightTime += newSol.hStart - drone.hDep
    #
    #         constraints_nodes, constraints_edges = self.add_constraints(constraints_nodes, constraints_edges, drone, newSol)
    #
    #         return constraints_nodes, constraints_edges, newSol, delay
    #
    #     for dronesList in perms:
    #         constraintsNodes = self.initial_constraint_nodes
    #         constraintsEdges = self.initial_constraint_edges
    #     # constraintsNodes = {}
    #     # constraintsEdges = {}
    #         pathsChanged = []
    #         pathsUnchanged = []
    #         for d in dronesList:
    #             constraintsNodes, constraintsEdges, path, delay = new_path(d, constraintsNodes, constraintsEdges)
    #             if path != None :
    #                 # d.path = path
    #                 pathsChanged.append(path)
    #             else:
    #                 pathsUnchanged.append(d.path)
    #
    #         totalTime = 0
    #         if len(pathsChanged) >= 1:
    #             for p in pathsChanged:
    #                 totalTime += p.flightTime
    #             for p in pathsUnchanged:
    #                 totalTime += p.flightTime
    #         else:
    #             totalTime = best_time + 10
    #         if totalTime <= best_time:
    #             best_time = totalTime
    #             bestPerm = dronesList
    #
    #     constraintsNodes = {}
    #     constraintsEdges = {}
    #     for d in bestPerm:
    #         constraintsNodes, constraintsEdges, path, delay = new_path(d, constraintsNodes, constraintsEdges)
    #         if path != None:
    #             d.path = path
    #
    # def solve_cluster(self, model):
    #     perms = list(itertools.permutations(self.drones))
    #     print("number of permutations ", len(perms))
    #     bestTime = 100000
    #
    #     def new_path(drone, constraint_nodes, constraints_edges):
    #         found = False
    #         i = 0
    #         while not found:
    #             if drone.path.path[i] in self.nodesList:
    #                 dep = drone.path.path[i]
    #                 found = True
    #             else:
    #                 i += 1
    #
    #         i = list(drone.path.pathDict.values()).index(dep)
    #         timeDep = list(drone.path.pathDict.keys())[i]
    #
    #         path_solution = a2.Astar(model.graph, dep, drone.arr, drone, timeDep, constraint_nodes, constraints_edges)
    #         new_solution = None
    #         if path_solution is not None:
    #             if not drone.path.previousPath == []:
    #                 previous_path = drone.path.previousPath.path
    #             else :
    #                 previous_path = []
    #             # print(drone)
    #             # previous_path = drone.path.previousPath.path
    #             new_path = drone.path.path[:i].copy() + path_solution.path.copy()
    #             if drone.path.path != new_path and new_path != previous_path:
    #                 new_solution = pt.Path(drone.hDep, new_path.copy())
    #                 new_solution.previousPath = drone.path
    #                 new_solution.add_path(new_path.copy(), model.graph, drone)
    #                 new_solution.discretize_path(5, model.graph, drone)
    #                 new_solution.flight_time_and_distance(model.graph, drone)
    #         constraint_nodes, constraints_edges = self.add_constraints(constraint_nodes, constraints_edges, drone, new_solution)
    #         return constraint_nodes, constraints_edges, new_solution
    #
    #     for dronesList in perms:
    #         constraintsNodes = {}
    #         constraintsEdges = {}
    #         pathsChanged = []
    #         pathsUnchanged = []
    #         for d in dronesList:
    #             constraintsNodes, constraintsEdges, path = new_path(d, constraintsNodes, constraintsEdges)
    #             if path != None:
    #                 pathsChanged.append(path)
    #             else:
    #                 pathsUnchanged.append(d.path)
    #         totalTime = 0
    #         if len(pathsChanged) != 0:
    #             for p in pathsChanged:
    #                 totalTime += p.flightTime
    #             for p in pathsUnchanged:
    #                 totalTime += p.flightTime
    #         else:
    #             totalTime = bestTime + 10
    #         if totalTime <= bestTime:
    #             bestTime = totalTime
    #             bestPerm = dronesList
    #
    #     constraintsNodes = {}
    #     constraintsEdges = {}
    #     # TODO This raises an error if no path has been found in all permutations
    #     for d in bestPerm:
    #         constraintsNodes, constraintsEdges, path = new_path(d, constraintsNodes, constraintsEdges)
    #         if path != None:
    #             d.path = path
    #         # print(" Final paths : ", d.path.path)

    def solve_cluster_dual(self, model, max_permutations):
        """Solve the cluster by solving the problem sequentially over all possible permutations or at least a certain
        number"""
        # Contain all possible permutations of drones order
        possible_permutations_nb = math.factorial(len(self.drones))
        # print("number of permutations ", possible_permutations_nb)
        best_time = 100000

        # Compute the new best path for the drone taking into account the other drones paths as constraints
        def new_path_dual(current_drone, constraint_primal_dict):
            """Find the new shortest path for the current drone, taking into account the constraints added
            by the already previously computed drones
            constraint_primal_dict = {dual_node : [arrival_time, next_node, next_node_time, drone]}
            If the drone actually juste pass by the first node but doesn't exit at the ending one, only the arrival time
            is filled"""
            found = False
            i = 0
            # Searching for the current node to be set as departure node
            # TODO bof ?
            while not found:
                if current_drone.path_object.path[i] in self.nodesList:
                    departure_node = current_drone.path_object.path[i]
                    found = True
                else:
                    i += 1
            i = list(current_drone.path_object.path_dict.values()).index(departure_node)
            departure_time = list(current_drone.path_object.path_dict.keys())[i]
            # Create the dual departure and arrival for the A* dual algorithm
            dep_dual = ("S" + departure_node, departure_node)
            arr_dual = (drone.arr, drone.arr + "T")
            # Computing shortest path from the first node in the cluster to the arrival node
            path_solution = a2.astar_dual(model, dep_dual, arr_dual, drone, departure_time, constraint_primal_dict)
            new_solution = None
            # If a solution has been found
            if path_solution is not None:
                # Checking if there is a previous path already saved
                if not current_drone.path_object.previous_path == []:
                    previous_path = current_drone.path_object.previous_path.path
                else:
                    previous_path = []
                # The path is modified starting from the first waypoint taken into account in the cluster
                new_path = current_drone.path_object.path[:i].copy() + path_solution.path.copy()
                # Checking that the new path is not the same as the last one or the current one
                if current_drone.path_object.path != new_path and new_path != previous_path:
                    new_solution = pt.Path(current_drone.dep_time, new_path.copy())
                    new_solution.previous_path = current_drone.path_object
                    new_solution.set_path(new_path.copy(), model.graph, model.graph_dual, current_drone)
                    new_solution.discretize_path(5, model.graph, current_drone)
                    new_solution.flight_time_and_distance(model.graph, current_drone)
            # Adding the constraints, only change anything if the path has changed
            constraint_primal_dict = add_constraints_dual(constraint_primal_dict, current_drone, new_solution)
            # print(constraint_dict)
            return constraint_primal_dict, new_solution

        # Looking for the best permutation or at least one close enough to the best time
        permutation_count = 0
        # TODO ameliorer les permutations pour tester au moins un fois chaque avion en premier
        while permutation_count < min(possible_permutations_nb, max_permutations):
            drones_list = self.drones.copy()
            random.shuffle(drones_list)
            permutation_count += 1
            constraint_dict = {}
            paths_changed = []
            paths_unchanged = []
            # Computing the best path for each drone sequentially (each drone take the already computed ones as
            # constraints)
            for drone in drones_list:
                constraint_dict, path = new_path_dual(drone, constraint_dict)
                # Separate the changed and unchanged path
                if path is not None:
                    paths_changed.append(path)
                else:
                    paths_unchanged.append(drone.path_object)
            total_time = 0

            # Compute the total flight time using this permutation
            if len(paths_changed) != 0:
                for p in paths_changed:
                    total_time += p.flightTime
                for p in paths_unchanged:
                    total_time += p.flightTime
            else:
                total_time = best_time + 10
            # Check if the current permutation is better than the current best one
            if total_time <= best_time:
                best_time = total_time
                best_permutation = drones_list
            else:
                best_permutation = None

        # After iterating over the permutations we use the best found permutation and do the process
        # one last time to save the best paths found
        constraint_dict = {}
        if best_permutation is not None:
            for drone in best_permutation:
                constraint_dict, path = new_path_dual(drone, constraint_dict)
                if path is not None:
                    drone.path_object = path
        # else:
        #     print("No solutions found")


# TODO ce serait plus logique de donner un path dans tout les cas et de checker dans le code avant si il y en a un et donner direct celui du drone si besoin
def add_constraints_dual(constraint_dict, drone, path):
    """Take the path of drone and turn it into constraints to add to the constraint_dict
    constraint format is { node : [[time_at_node, next_node, time_at_next_node, drone], [...]] }"""
    # The constraint_dict is modified in here as it is mutable
    if path is None:
        path_to_use = drone.path_object
    else:
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
