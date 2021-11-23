import Node as nd
import networkx as nx
import Path as pt
import tools


def astar_dual(model, dep_node_id, arr_node_id, drone, departure_time, primal_constraint_nodes_dict=None):
    # model.countAstar += 1
    # print(model.countAstar)
    graph_dual = model.graph_dual
    # TODO TOUT CHANGER LES COST POUR TRAVAILLER QUE AVEC DES TEMPS
    if primal_constraint_nodes_dict is None:
        primal_constraint_nodes_dict = dict()
    current_node = nd.Node(dep_node_id)
    current_node.cost = 0
    # Everytime a node time is mentioned it's equivalent to the time at the first node of the dual node in the primal
    # graph
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
            # If current node was AB and neighbor is BC, the time we set here is the arrival time at B taking into
            # account pre and post turn added cost
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
            _cost = graph_dual.edges[dual_edge]["length"] - graph_dual.edges[dual_edge]["post_turn_cost"]
            # This is the arrival time at B following the previous example
            enter_time_drone1 = current_node.time + _cost/drone.speed
            #TODO vis a vis des distances de secu il faudra prendre en compte le post turn cost pour attendre qu'on se soit eloigné assew

        # Pour s'engager sur BC on veut verifier que B est libre et qu'on a pas croisé ou depassé de drones sur AB

        node_primal_1, node_primal_2 = current_node.id[0], current_node.id[1]

        if node_primal_2 in primal_constraint_nodes_dict:
            for constraint in primal_constraint_nodes_dict[node_primal_2]:
                enter_time_drone2, next_constrained_node, exit_time_drone2, drone2 = constraint
                if drone.flight_number != drone2.flight_number:
                    t_safety = model.protection_area / min(drone2.speed, drone.speed)

                    # Check that the node is available
                    if abs(enter_time_drone2 - enter_time_drone1) < t_safety:
                        neighbor_to_be_added = False

                    # Check the drone didn't cross another one the edge
                    if node_primal_1 == next_constrained_node:
                        interval1 = [current_node.time, enter_time_drone1]
                        interval2 = [enter_time_drone2, exit_time_drone2]
                        if tools.intersection(interval1, interval2) is not None:
                            neighbor_to_be_added = False

        # Check the drone didn't pass another on the edge
        if node_primal_1 in primal_constraint_nodes_dict:
            for constraint in primal_constraint_nodes_dict[node_primal_1]:
                enter_time_drone2, next_constrained_node, exit_time_drone2, drone2 = constraint
                if drone.flight_number != drone2.flight_number:
                    if next_constrained_node == node_primal_2:
                        if current_node.time < enter_time_drone2 and not enter_time_drone1 < exit_time_drone2:
                            neighbor_to_be_added = False
                        if enter_time_drone2 < current_node.time and not exit_time_drone2 < enter_time_drone1 :
                            neighbor_to_be_added = False

        if neighbor_to_be_added:
            neighbors.append(neighbor)
    return neighbors

