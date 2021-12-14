import Node as nd
import networkx as nx
import Path as pt
import tools
import Model


def astar_dual(model, dep_node_id, arr_node_id, drone, departure_time, primal_constraint_nodes_dict=None):
    graph_dual = model.graph_dual
    closed_nodes_list = []
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
        closed_nodes_list.append(current_node.id)
        # TERMINATION CONDITION : Having reached the arrival node
        if current_node.id == arr_node_id:
            solution_dual_path = current_node.path()
            # Turning the path back in the normal graph path
            shortest_path = pt.Path(drone.dep_time, [node.id[1] for node in solution_dual_path[:-1]])
            return shortest_path
        neighbors = get_available_neighbors_dual(current_node, primal_constraint_nodes_dict, drone, model)
        for neighbor in neighbors:
            edge = (current_node.id, neighbor)
            speed_on_neighbor = drone.cruise_speed
            if graph_dual.edges[current_node.id, neighbor]["is_turn"]:
                speed_on_neighbor = drone.turn_speed
            if current_node.parent is not None:
                if graph_dual.edges[current_node.parent.id, current_node.id]["is_turn"]:
                    speed_on_neighbor = drone.turn_speed
            # If current node was AB and neighbor is BC, the time we set here is the arrival time at B taking into
            # account pre and post turn added cost
            new_time = current_node.time + graph_dual.edges[edge]["length"] / speed_on_neighbor
            new_cost = current_node.cost + graph_dual.edges[edge]["length"]
            neighbor_in_priority_queue, neighbor = node_in_list(neighbor, priority_queue)
            # RQ : le fait de pas etre dans closed nodes on pourrait le faire avant les calculs de neighbor
            if not neighbor_in_priority_queue and neighbor.id not in closed_nodes_list:
                priority_queue.append(neighbor)
            # EFFET DE BORD pour modifier la valeur de neighbor dans la liste priority_queue
            if neighbor.time >= new_time:
                neighbor.time = new_time
                neighbor.cost = new_cost
                # RQ on peut ajouter le cout du neighbor avec le virage a l'heuristic
                neighbor.heuristic = neighbor.dist_to_node(arr_node_id, graph_dual)
                neighbor.parent = current_node
        priority_queue.sort(key=lambda x: x.f())


#
# def astar_dual_no_limit(model, dep_node_id, arr_node_id, drone, departure_time, primal_constraint_nodes_dict=None):
#     """A star without a closed node list and no restriction to add in open nodes"""
#     # model.countAstar += 1
#     # print(model.countAstar)
#     graph_dual = model.graph_dual
#     # closed_nodes_list = []
#     if primal_constraint_nodes_dict is None:
#         primal_constraint_nodes_dict = dict()
#     current_node = nd.Node(dep_node_id)
#     current_node.cost = 0
#     # Everytime a node time is mentioned it's equivalent to the time at the first node of the dual node in the primal
#     # graph
#     current_node.time = departure_time
#     current_node.heuristic = current_node.dist_to_node(arr_node_id, graph_dual)
#     priority_queue = [current_node]
#
#     while len(priority_queue) > 0:
#         current_node = priority_queue.pop(0)
#         # closed_nodes_list.append(current_node.id)
#         # TERMINATION CONDITION : Having reached the arrival node
#         if current_node.id == arr_node_id:
#             solution_dual_path = current_node.path()
#             # Turning the path back in the normal graph path
#             shortest_path = pt.Path(drone.dep_time, [node.id[1] for node in solution_dual_path[:-1]])
#             return shortest_path
#         neighbors = get_available_neighbors_dual(current_node, primal_constraint_nodes_dict, drone, model)
#         for neighbor_id in neighbors:
#             edge = (current_node.id, neighbor_id)
#             speed_on_neighbor = drone.cruise_speed
#             if graph_dual.edges[current_node.id, neighbor_id]["is_turn"]:
#                 speed_on_neighbor = drone.turn_speed
#             if current_node.parent is not None:
#                 if graph_dual.edges[current_node.parent.id, current_node.id]["is_turn"]:
#                     speed_on_neighbor = drone.turn_speed
#             # If current node was AB and neighbor is BC, the time we set here is the arrival time at B taking into
#             # account pre and post turn added cost
#             new_time = current_node.time + graph_dual.edges[edge]["length"] / speed_on_neighbor
#             new_cost = current_node.cost + graph_dual.edges[edge]["length"]
#             neighbor = nd.Node(neighbor_id)
#             priority_queue.append(neighbor)
#             # neighbor_in_priority_queue, neighbor = node_in_list(neighbor, priority_queue)
#             # # RQ : le fait de pas etre dans closed nodes onpourrait le faire avant les calculs de neighbor
#             # if not neighbor_in_priority_queue and neighbor.id not in closed_nodes_list:
#             #     priority_queue.append(neighbor)
#             # EFFET DE BORD pour modifier la valeur de neighbor dans la liste priority_queue
#             if neighbor.time >= new_time:
#                 neighbor.time = new_time
#                 neighbor.cost = new_cost
#                 # RQ on peut ajouter le cout du neighbor avec le virage a l'heuristic
#                 neighbor.heuristic = neighbor.dist_to_node(arr_node_id, graph_dual)
#                 neighbor.parent = current_node
#         priority_queue.sort(key=lambda x: x.f())


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
        # print("node succesors :", list(graph_dual.successors(current_node.id)))
        neighbor_to_be_added = True
        neighbor_enter_node = neighbor[0]
        neighbor_exit_node = neighbor[1]
        dual_edge = (current_node.id, neighbor)
        current_edge = current_node.id
        # We need the current drone speed after exiting the entry node of the neighbor (= entering the neighbor edge)
        # along with it's arrival time at the entry node
        # To have the entry time we need the speed on the last edge
        # Ex : if current node is AB and neighbor is BC, the time of arrival at BC would be
        # time at A + time to travel through AB with the right speed to take the turn
        # We can't check if BC is actually feasible now because we don't know if the node C will be a turn or not
        # If the current node is a source the length is 0 so entry time is the same as current_node.time
        # So we also need the drone speed on the last edge

        # Determine the drone time of arrival on the current edge :
        # Was the last node a turn point :

        neighbor_edge_entry_time = None

        if current_edge[0][0] == "S":
            current_edge_length = 0
        else:
            current_edge_length = graph_primal.edges[current_edge]["length"]
        if current_node.parent is not None:
            if graph_dual.edges[current_node.parent.id, current_node.id]["is_turn"]:
                neighbor_edge_entry_time = current_node.time + current_edge_length / drone.turn_speed
        # Is the coming node a turning point
        if graph_dual.edges[current_node.id, neighbor]["is_turn"] and not current_edge_length == 0:
            if current_edge_length < model.protection_area + drone.braking_distance:
                neighbor_edge_entry_time = current_node.time + current_edge_length / drone.turn_speed
            else:
                # Check that the last node wasn't a turn point
                if neighbor_edge_entry_time is None:
                    neighbor_edge_entry_time = (current_node.time
                                                + (current_edge_length - drone.braking_distance)/drone.cruise_speed
                                                + 2*drone.braking_distance / (drone.cruise_speed - drone.turn_speed))
        # If time is still None then neither the previous node was a turn or the coming one
        if neighbor_edge_entry_time is None:
            neighbor_edge_entry_time = current_node.time + current_edge_length / drone.cruise_speed

        # Drone speed on the neighbor edge :
        # We can't know if the node at the end of the neighbor edge is a turnpoint, so if the edge is too short we have
        # to take the turn_speed as speed on the edge to work in the worst case too.
        drone1_speed_on_neighbor = None

        if not neighbor[1][-1] == "T":
            neighbor_length = graph_primal.edges[neighbor]["length"]
        else:
            neighbor_length = 0
        if graph_dual.edges[current_node.id, neighbor]["is_turn"]:
            drone1_speed_on_neighbor = drone.turn_speed
        else:
            if neighbor_length > model.protection_area + drone.braking_distance and not neighbor_length == 0:
                drone1_speed_on_neighbor = drone.cruise_speed
            else:
                drone1_speed_on_neighbor = drone.turn_speed
        # if drone1_speed_on_neighbor is None:
        #     drone1_speed_on_neighbor = drone.cruise_speed

        # Is there a conflict on the node or on the edge by going both ways ?
        if neighbor[0] in primal_constraint_nodes_dict:
            for constraint in primal_constraint_nodes_dict[neighbor[0]]:
                enter_time_drone2, next_constrained_node, exit_time_drone2, drone2 = constraint
                # Find the other drone speed when passing the entry node of the edge
                drone2_speed_on_neighbor = None
                drone2_path = drone2.path_object
                drone2_time_stamps = sorted(drone2_path.path_dict.keys())
                d2_entry_time_index = drone2_time_stamps.index(enter_time_drone2)
                # Check if the node was a turn
                d2_node_minus1 = None
                # Check it's at least the second node of d2 path and not it's last one otherwise this can't be a turn
                if d2_entry_time_index > 0 and not next_constrained_node == neighbor[0]:
                    d2_node_minus1 = drone2_path.path_dict[drone2_time_stamps[d2_entry_time_index-1]]
                    if graph_dual.edges[(d2_node_minus1, neighbor[0]), (neighbor[0], next_constrained_node)]["is_turn"]:
                        drone2_speed_on_neighbor = drone2.turn_speed
                # Check if next node is a turn
                if d2_entry_time_index + 2 < len(drone2_path.path) - 1:
                    d2_node_plus2 = drone2_path.path_dict[drone2_time_stamps[d2_entry_time_index+2]]
                    if graph_dual.edges[(neighbor[0], next_constrained_node), (next_constrained_node, d2_node_plus2)]["is_turn"]:
                        if neighbor_length < model.protection_area + drone2.braking_distance and not neighbor_length == 0:
                            drone2_speed_on_neighbor = drone2.turn_speed
                        else:
                            if drone2_speed_on_neighbor is not None:
                                drone2_speed_on_neighbor = drone2.cruise_speed
                if drone2_speed_on_neighbor is None:
                    drone2_speed_on_neighbor = drone2.cruise_speed
                if Model.check_conflict_on_node(neighbor_edge_entry_time, enter_time_drone2, drone1_speed_on_neighbor, drone2_speed_on_neighbor, model.protection_area) is not None:
                    neighbor_to_be_added = False
                # Check if there was a drone going the other way on the edge
                if current_node.id[0] == next_constrained_node:
                    interval1 = [current_node.time, neighbor_edge_entry_time]
                    interval2 = [enter_time_drone2, exit_time_drone2]
                    if tools.intersection(interval1, interval2) is not None:
                        neighbor_to_be_added = False
        # Is there a conflict on the edge going the same way
        if current_node.id[0] in primal_constraint_nodes_dict:
            for constraint in primal_constraint_nodes_dict[current_node.id[0]]:
                enter_time_drone2, next_constrained_node, exit_time_drone2, drone2 = constraint
                if current_node.id[1] == next_constrained_node:
                    if current_node.time < enter_time_drone2 and not neighbor_edge_entry_time < exit_time_drone2:
                        neighbor_to_be_added = False
                    if current_node.time > enter_time_drone2 and not neighbor_edge_entry_time > exit_time_drone2:
                        neighbor_to_be_added = False
        if neighbor_to_be_added:
            neighbors.append(neighbor)
    return neighbors

