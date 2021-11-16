import Node as nd
import networkx as nx
import Path as pt
import tools


def astar_dual(model, dep_node_id, arr_node_id, drone, departure_time, primal_constraint_nodes_dict=None):
    graph_dual = model.graph_dual
    if primal_constraint_nodes_dict is None:
        primal_constraint_nodes_dict = dict()
    current_node = nd.Node(dep_node_id)
    current_node.cost = 0
    current_node.time = departure_time
    current_node.heuristic = current_node.dist_to_node(arr_node_id, graph_dual)
    priority_queue = [current_node]
    while len(priority_queue) > 0:
        current_node = priority_queue.pop(0)
        # TERMINATION CONDITION : Having reached the arrival node
        if current_node.id == arr_node_id:
            solution_dual_path = current_node.path()
            # Turning the path back in the normal graph path
            shortest_path = pt.Path(drone.dep_time, [node.id[1] for node in solution_dual_path[:-1]])
            return shortest_path
        neighbors = get_available_neighbors_dual(current_node, primal_constraint_nodes_dict, drone, model)
        for neighbor in neighbors:
            edge = (current_node.id, neighbor)
            new_time = current_node.time + graph_dual.edges[edge]["length"] / drone.speed
            new_cost = current_node.cost + graph_dual.edges[edge]["length"]
            neighbor_in_priority_queue, neighbor = node_in_list(neighbor, priority_queue)
            if not neighbor_in_priority_queue:
                priority_queue.append(neighbor)
            # EFFET DE BORD pour modifier la valeur de neighbor dans la liste priority_queue
            if neighbor.time >= new_time:
                neighbor.time = new_time
                neighbor.cost = new_cost
                neighbor.heuristic = neighbor.dist_to_node(arr_node_id, graph_dual)
                neighbor.parent = current_node
        priority_queue.sort(key=lambda x: x.f())


def node_in_list(node_id, node_list):
    """Returns whether a node is in a given list and the node object associated to the nodeId"""
    for node in node_list:
        if node.id == node_id:
            return True, node
    return False, nd.Node(node_id)


def get_available_neighbors_dual(current_node, primal_constraint_nodes_dict, drone, model):
    """Return the list of available neighbors to visit from the current_node taking into account the current
    constraints (primal_constraint_nodes_dict)"""
    neighbors = []
    graph_dual, graph_primal = model.graph_dual, model.graph
    # Since it's a directed graph, graph.successors is used
    for neighbor in graph_dual.successors(current_node.id):
        neighbor_to_be_added = True
        neighbor_enter_node = neighbor[0]
        neighbor_exit_node = neighbor[1]
        dual_edge = (current_node.id, neighbor)

        # Setting the right time for the neighbor
        # Ex : if current node is AB and neighbor is BC, the time of arrival at BC would be
        # time at A + time to travel through AB + pre_turn cost to BC
        # If the current node is a source the length is 0
        if current_node.id[0][0] == "S":
            enter_time_drone1 = current_node.time
        else:
            _cost = graph_dual.edges[dual_edge]["length"] - graph_dual.edges[dual_edge]["pre_turn_cost"]
            enter_time_drone1 = current_node.time + _cost/drone.speed
        # Finding the neighbor length for cost later
        # TODO ici on peut pas savoir exactement a quelle heure sort le drone du neighbor si il l' emprunte
        if neighbor_enter_node[0] == "S" or neighbor_exit_node[-1] == "T":
            length = 0
        else:
            length = graph_primal.edges[neighbor]["length"]
        # Compute the expected exit time, assuming the drone fly at constant speed
        post_cost = graph_dual.edges[dual_edge]["post_turn_cost"]
        exit_time_drone1 = enter_time_drone1 + (length + post_cost)/drone.speed

        # Checking all constrained node
        # Constraint format = {node : [time_node, next_node, time_next_node, drone]}
        for constrained_node in primal_constraint_nodes_dict.keys():
            for constraint in primal_constraint_nodes_dict[constrained_node]:
                enter_time_drone2, next_constrained_node, exit_time_drone2, drone2 = constraint
                if drone.flight_number != drone2.flight_number:
                    # TODO Using the speed of the last drone that passed by the node would be more accurate
                    t_safety = model.protection_area / min(drone2.speed, drone.speed)

                    # A drone enter at the current node
                    if constrained_node == neighbor[0]:
                        # Check that the drone only use it when the other drone has had time to leave
                        if enter_time_drone2 - t_safety < enter_time_drone1 < enter_time_drone2 + t_safety:
                            neighbor_to_be_added = False

                    # A drone exit at the current node
                    if next_constrained_node == neighbor[0]:
                        # Check that the drone only use it when the other drone has had time to leave
                        if exit_time_drone2 - t_safety < enter_time_drone1 < exit_time_drone2 + t_safety:
                            neighbor_to_be_added = False
                    # TODO On a des pb vis a vis des temps de passage sur les noeuds de sortie car on connait pas encore le next node pour le drone actuel
                    # A drone uses the same street the same way
                    if constrained_node == neighbor[0] and next_constrained_node == neighbor[1]:
                        # Check that they exit in the same order and not too close
                        if exit_time_drone2 - t_safety < exit_time_drone1 < exit_time_drone2 + t_safety:
                            neighbor_to_be_added = False
                        # # If entering first but exiting last
                        if enter_time_drone1 < enter_time_drone2 and exit_time_drone1 > exit_time_drone2:
                            neighbor_to_be_added = False
                        # If entering last but exiting first
                        if enter_time_drone1 > enter_time_drone2 and exit_time_drone1 < exit_time_drone2:
                            neighbor_to_be_added = False

                    # A drone uses the same street but the other way
                    if constrained_node == neighbor[1] and next_constrained_node == neighbor[0]:
                        # Check that they don't cross
                        interval1 = [enter_time_drone1 - t_safety, exit_time_drone1 + t_safety]
                        interval2 = [enter_time_drone2 - t_safety, exit_time_drone2 + t_safety]
                        if tools.intersection(interval1, interval2) is not None:
                            neighbor_to_be_added = False

        if neighbor_to_be_added:
            neighbors.append(neighbor)
    return neighbors


# def get_neighbors(currentNode, constraintsNodes, constraintsEdges, drone, graph):
#     neighbors = []
#     for neighbor in graph.adj[currentNode.id]:
#         edgeAvailable = True
#         nodeAvailable = True
#
#         edge = (currentNode.id, neighbor)
#         currentTime = currentNode.time
#         newTime = currentTime + graph.edges[edge]["length"] / drone.speed
#
#         # Availability of edge
#         if (edge[1], edge[0]) in constraintsEdges or edge in constraintsEdges:
#             for (timeStartConstraint, timeEndConstraint) in constraintsEdges[edge]:
#                 if timeStartConstraint <= currentTime <= timeEndConstraint or timeStartConstraint <= newTime <= timeEndConstraint:
#                     edgeAvailable = False
#
#         # Availability of the node
#         if neighbor in constraintsNodes:
#             for (timeStartConstraint, timeEndConstraint) in constraintsNodes[neighbor]:
#                 if timeStartConstraint <= newTime <= timeEndConstraint:
#                     nodeAvailable = False
#
#         if edgeAvailable and nodeAvailable:
#             neighbors.append(neighbor)
#     return neighbors
#
#
# def Astar(graph:nx.Graph, depId:str, arrId:str, drone, timeDep, constraintsNodes={}, constraintsEdges={}):
#     currentNode = nd.Node(depId)
#     currentNode.cost = 0
#     currentNode.time = timeDep
#     currentNode.heuristic = currentNode.dist_to_node(arrId, graph)
#     priorityQueue = [currentNode]
#     while len(priorityQueue) > 0:
#         currentNode = priorityQueue.pop(0)
#         if currentNode.id == arrId:
#             sol = currentNode.path()
#             path = pt.Path(drone.hDep, [node.id for node in sol])
#             # print("non dual",path.path)
#             return path
#         neighbors = get_neighbors(currentNode, constraintsNodes, constraintsEdges, drone, graph)
#         for v in neighbors:
#             edge = (currentNode.id, v)
#             newTime = currentNode.time + graph.edges[edge]["length"] / drone.speed
#             newCost = currentNode.cost + graph.edges[edge]["length"]
#             vInPriorityQueue, v = node_in_list(v, priorityQueue)
#             if not vInPriorityQueue:
#                 priorityQueue.append(v)
#             if v.time >= newTime:
#                 v.time = newTime
#                 v.cost = newCost
#                 v.heuristic = v.dist_to_node(arrId, graph)
#                 v.parent = currentNode
#         priorityQueue.sort(key=lambda x: x.f())

