# import tools as tools
import Drone


over_estimate_turn_factor = 1.2


class Path:
    def __init__(self, hStart, path=None):
        # Path contain the pth as a list of node
        if path is None:
            path = []
        self.path = path
        # Previous path is used to store the last path that was taken
        # TODO passer les attributs en private et forcer l'utilisation des setters et getters
        self.previous_path = []
        # Path dict contain a path with the time as keys and the node as value
        self.path_dict = {}
        self.previous_node_dict = dict()
        self.next_node_dict = dict()
        self.separation_dict = dict()
        self.edge_dict = dict()
        # edge_middle_dict = {edge_id : t_middle, v_middle}
        self.edge_middle_dict = dict()
        # Store the fixed speed at this node, None if there's none
        self.fixed_speed_wpt = []
        # Speed ate time stamp after the node
        self.speed_time_stamps = dict()
        # The discretized version of the path with time as key and position x,y as value
        self.path_dict_discretized = {}
        self.delay = {}
        self.dep_time = hStart
        self.arr_time = None
        self.flightTime = 0
        self.flightDistance = 0

    def set_path(self, new_path, model):
        """Adds a new path to the drone, in terms of waypoints (self.path) and waypoints
                and timestamps (self.pathDict) using the dual graph to set the timestamps with
                the added turning times into account"""
        graph_dual = model.graph_dual
        graph = model.graph
        self.path_dict = {}
        self.path = new_path
        t = self.dep_time
        self.speed_time_stamps[t] = Drone.speeds_dict["cruise"]
        self.fixed_speed_wpt.append(None)
        self.path_dict[t] = new_path[0]
        # drone_speed = drone.cruise_speed
        # TODO EDGE DICT {edge id : (t_entree, t_sortie)}
        speed_at_current_node = Drone.speeds_dict["cruise"]
        # TODO AJOUTER t supplementaire depart depuis milieu edge depart

        for index in range(1, len(new_path)):
            current_edge = (new_path[index-1], new_path[index])
            # length = graph_dual.edges[current_edge_dual]["length"]
            length = graph.edges[current_edge]["length"]
            fixed_speed = None

            # Check if last node was a turn
            last_node_was_turn = False
            speed_at_previous_node = None
            if index > 2:
                previous_edge_dual = ((new_path[index-2], new_path[index-1]), (new_path[index-1], new_path[index]))
                angle = graph_dual.edges[previous_edge_dual]["angle"]
                speed_at_previous_node = Drone.return_speed_from_angle(angle)
                if speed_at_previous_node != Drone.speeds_dict["cruise"]:
                    last_node_was_turn = True

            # Check if this node is a turn
            current_node_is_turn = False
            if index < len(new_path) - 1:
                current_edge_dual = ((new_path[index - 1], new_path[index]), (new_path[index], new_path[index + 1]))
                angle = graph_dual.edges[current_edge_dual]["angle"]
                speed_at_current_node = Drone.return_speed_from_angle(angle)
                if speed_at_current_node != Drone.speeds_dict["cruise"]:
                    current_node_is_turn = True

            # Determine travel time on edge and arrival time at the node along with the
            # 2 separation time (before and after the node)
            # If both nodes were turns
            if last_node_was_turn and current_node_is_turn:
                accel_distance_last_node = Drone.return_braking_distance(speed_at_previous_node, Drone.speeds_dict["cruise"])
                accel_distance_current_node = Drone.return_braking_distance(speed_at_current_node, Drone.speeds_dict["cruise"])
                # TODO a preciser
                if length > accel_distance_last_node + accel_distance_current_node:
                    travel_time = (Drone.return_accel_time(speed_at_previous_node) + Drone.return_accel_time(speed_at_current_node)) * over_estimate_turn_factor
                    travel_time += (length - accel_distance_last_node - accel_distance_current_node) / Drone.speeds_dict["cruise"]
                else:
                    # If there's not enough space to accelerate fully, then speed will be set to the speed of the coming turn
                    inter_accel_distance = Drone.return_braking_distance(speed_at_previous_node, speed_at_current_node)
                    travel_time = (Drone.return_accel_time(speed_at_current_node, speed_at_previous_node))*over_estimate_turn_factor
                    travel_time += (length - inter_accel_distance) / speed_at_current_node
                    fixed_speed = speed_at_current_node

                if model.protection_area < accel_distance_current_node:
                    t_sep = Drone.return_accel_time(speed_at_current_node)
                else:
                    t_sep = Drone.return_accel_time(speed_at_current_node) + (model.protection_area - accel_distance_current_node)/speed_at_current_node

            # If the node is  a turn point
            elif current_node_is_turn and not last_node_was_turn:
                accel_distance_current_node = Drone.return_braking_distance(speed_at_current_node, Drone.speeds_dict["cruise"])
                if length > accel_distance_current_node:
                    travel_time = (Drone.return_accel_time(speed_at_current_node)) * over_estimate_turn_factor
                    travel_time += (length - accel_distance_current_node)/Drone.speeds_dict["cruise"]
                    if model.protection_area < accel_distance_current_node:
                        t_sep = Drone.return_accel_time(speed_at_current_node)
                    else:
                        t_sep = Drone.return_accel_time(speed_at_current_node) + (model.protection_area - accel_distance_current_node)/Drone.speeds_dict["cruise"]
                # If there's not enough distance to brake
                else:
                    travel_time = Drone.return_accel_time(speed_at_current_node)
                    t_sep = Drone.return_accel_time(speed_at_current_node)

            # If the last node was a turn but not the coming one
            elif last_node_was_turn and not current_node_is_turn:
                accel_distance_last_node = (Drone.return_braking_distance(speed_at_previous_node, Drone.speeds_dict["cruise"])) *over_estimate_turn_factor
                travel_time = Drone.return_accel_time(speed_at_previous_node)
                travel_time += (length - accel_distance_last_node)/Drone.speeds_dict["cruise"]
                t_sep = model.protection_area / Drone.speeds_dict["cruise"]

            # If neither nodes are turning points
            else:
                travel_time = length / Drone.speeds_dict["cruise"]
                t_sep = model.protection_area / Drone.speeds_dict["cruise"]

            self.fixed_speed_wpt.append(fixed_speed)

            previous_t = t
            t += travel_time
            self.path_dict[t] = new_path[index]
            self.speed_time_stamps[t] = speed_at_current_node
            self.separation_dict[t] = (t_sep, t_sep)
            self.edge_dict[current_edge] = (previous_t, t)
            t_at_middle = (t+previous_t)/2
            if speed_at_previous_node is not None:
                speed_middle = (speed_at_previous_node + speed_at_current_node)/2
            else:
                speed_middle = speed_at_current_node
            self.edge_middle_dict[current_edge] = (t_at_middle, speed_middle)

        self.arr_time = t

        # creating next and previous node path :
        sorted_time_stamps = sorted(self.path_dict.keys())
        for i in range(0, len(sorted_time_stamps) - 1):
            t = sorted_time_stamps[i]
            node = self.path_dict[t]
            if i > 0:
                previous_time = sorted_time_stamps[i - 1]
                previous_node = self.path_dict[previous_time]
                self.previous_node_dict[(t, node)] = (previous_time, previous_node)
            if i < len(sorted_time_stamps) - 1:
                next_time = sorted_time_stamps[i + 1]
                next_node = self.path_dict[next_time]
                self.next_node_dict[(t, node)] = (next_time, next_node)
        # print(len(new_path))
        # print(len(self.path_dict.keys()))
        # print(new_path)
        # print(self.path_dict)

    # def set_path(self, new_path, graph, graph_dual, drone):
    #     """Adds a new path to the drone, in terms of waypoints (self.path) and waypoints
    #     and timestamps (self.pathDict) using the dual graph to set the timestamps with
    #     the added turning times into account"""
    #     self.path_dict = {}
    #     self.path = new_path
    #     t = self.dep_time
    #     self.speed_time_stamps[t] = Drone.speeds_dict["cruise"]
    #     self.path_dict[t] = new_path[0]
    #     # TODO implement case where path length = 2
    #     # drone_speed = drone.cruise_speed
    #     self.edge_path = []
    #     drone_speed_after_node = Drone.speeds_dict["cruise"]
    #     for node_index in range(len(new_path)-1):
    #         self.edge_path.append((new_path[node_index], new_path[node_index+1]))
    #     for node_index in range(1, len(new_path)-1):
    #         drone_speed_after_node = Drone.speeds_dict["cruise"]
    #         # print(new_path)
    #         # To have the speed on the edge we need to know if the last node was a turn and if the next one is
    #         previous_edge, current_edge, next_edge = None, None, None
    #         # drone_speed = drone.cruise_speed
    #         # Edge dual is used to add the pre and post turn cost
    #         edge_dual = ((new_path[node_index-1], new_path[node_index]),
    #                      (new_path[node_index], new_path[node_index+1]))
    #         # Determine travel cost to the node (remove the post turn cost)
    #         try:
    #             cost = graph_dual.edges[edge_dual]["length"]
    #         except:
    #             print(new_path)
    #             raise Exception
    #         # If the edge length is more than turn_distance then we have to take into account the deceleration
    #         # and the phase where the drone was going at his normal speed
    #         if node_index > 2:
    #             previous_edge = (new_path[node_index-2], new_path[node_index-1])
    #         if node_index > 1:
    #             current_edge = (new_path[node_index-1], new_path[node_index])
    #         if node_index + 1 < len(new_path) - 1:
    #             next_edge = (new_path[node_index], new_path[node_index+1])
    #         # TODO ajouter le cas du fly by
    #         # If the last node passed was a turn point
    #         if current_edge is not None and previous_edge is not None:
    #             if graph_dual.edges[previous_edge, current_edge]["is_turn"]:
    #                 angle = graph_dual.edges[previous_edge, current_edge]["angle"]
    #                 turning_speed = Drone.return_speed_from_angle(angle)
    #                 # drone_speed = drone.turn_speed
    #                 # drone_speed = turning_speed
    #                 drone_speed_after_node = turning_speed
    #                 t += cost / turning_speed
    #             # If last node wasn't a turn point, the next one may be one
    #             elif current_edge is not None and next_edge is not None:
    #                 if graph_dual.edges[current_edge, next_edge]["is_turn"]:
    #                     angle = graph_dual.edges[current_edge, next_edge]["angle"]
    #                     turning_speed = Drone.return_speed_from_angle(angle)
    #                     braking_distance = Drone.return_braking_distance(Drone.speeds_dict["cruise"], turning_speed)
    #                     # drone_speed = drone.turn_speed
    #                     if graph.edges[current_edge]["length"] < braking_distance:
    #                         t += cost / turning_speed
    #                         drone_speed_after_node = turning_speed
    #                     else:
    #                         t += (graph.edges[current_edge]["length"] - braking_distance) / Drone.speeds_dict["cruise"]
    #                         # Adding the added time with constant deceleration
    #                         t += braking_distance / ((Drone.speeds_dict["cruise"] + turning_speed) / 2)
    #                 else:
    #                     t += cost / Drone.speeds_dict["cruise"]
    #             else:
    #                 t += cost / Drone.speeds_dict["cruise"]
    #         # If it's the first node and there is no previous node we still need to check the next one
    #         # Else if the next node is a turning point
    #         elif current_edge is not None and next_edge is not None:
    #             if graph_dual.edges[current_edge, next_edge]["is_turn"]:
    #                 angle = graph_dual.edges[current_edge, next_edge]["angle"]
    #                 turning_speed = Drone.return_speed_from_angle(angle)
    #                 braking_distance = Drone.return_braking_distance(Drone.speeds_dict["cruise"], turning_speed)
    #                 # drone_speed = drone.turn_speed
    #                 if graph.edges[current_edge]["length"] < braking_distance:
    #                     t += cost / turning_speed
    #                     drone_speed_after_node = turning_speed
    #                 else:
    #                     t += (graph.edges[current_edge]["length"] - braking_distance) / drone.cruise_speed
    #                     # Adding the added time with constant deceleration
    #                     t += braking_distance / ((drone.cruise_speed + turning_speed) / 2)
    #             else:
    #                 t += cost / Drone.speeds_dict["cruise"]
    #         # Else if there is no turning before or after
    #         else:
    #             t += cost/Drone.speeds_dict["cruise"]
    #
    #         self.path_dict[t] = new_path[node_index]
    #         self.speed_time_stamps[t] = drone_speed_after_node
    #         self.arr_time = t
    #     # Last node :
    #     # print(self.speed_time_stamps)
    #     try:
    #         cost = graph.edges[new_path[-2], new_path[-1]]["length"]
    #     except:
    #         print(len(new_path))
    #         raise Exception
    #     #TODO a verifier qu'il faut bien ajouter un dernier t parce que ça pourrait etre turn l'avant dernier
    #     t += cost/Drone.speeds_dict["cruise"]
    #     self.speed_time_stamps[t] = Drone.speeds_dict["cruise"]
    #     self.path_dict[t] = new_path[-1]
    #
    #     # creating next and previous node path :
    #     sorted_time_stamps = sorted(self.path_dict.keys())
    #     for i in range(0, len(sorted_time_stamps)-1):
    #         t = sorted_time_stamps[i]
    #         node = self.path_dict[t]
    #         if i > 0:
    #             previous_time = sorted_time_stamps[i-1]
    #             previous_node = self.path_dict[previous_time]
    #             self.previous_node_dict[(t, node)] = (previous_time, previous_node)
    #         if i < len(sorted_time_stamps) - 1:
    #             next_time = sorted_time_stamps[i+1]
    #             next_node = self.path_dict[next_time]
    #             self.next_node_dict[(t, node)] = (next_time, next_node)

    def flight_time_and_distance(self, graph):
        self.flightDistance = 0
        for i in range(len(self.path)-1):
            self.flightDistance += graph.edges[self.path[i], self.path[i+1]]['length']
        self.flightTime = self.speed_time_stamps[-1][0]
        for node in self.delay:
            self.flightTime += self.delay[node]

    # def discretize_path(self, dt, graph, drone):
    #     loop = True
    #     x0, y0 = graph.nodes[drone.dep]['x'], graph.nodes[drone.dep]['y']
    #     xEnd, yEnd = graph.nodes[drone.arr]['x'], graph.nodes[drone.arr]['y']
    #     edgeIndex = 0
    #     edge = [self.path[edgeIndex], self.path[edgeIndex+1]]
    #     i = 0
    #     t = self.dep_time
    #     self.path_dict_discretized = {t: (x0, y0)}
    #     if edge[0] in self.delay:
    #         c = self.delay[edge[0]]//dt
    #         for j in range(c):
    #             t += dt
    #             self.path_dict_discretized[t] = (x0, y0)
    #
    #     try:
    #         drone_speed = self.speed_time_stamps[0][1]
    #     except Exception:
    #         print(drone.flight_number)
    #         print(drone.path_object.path)
    #         print(drone.path_object.path_dict)
    #         print(drone.path_object.speed_time_stamps)
    #         raise Exception
    #     next_time_stamp = self.speed_time_stamps[0][0]
    #     time_stamp_index = 0
    #
    #     dist_to_travel = dt*drone_speed
    #     traveled = 0
    #     geometryList = graph.edges[edge]["geometry"]
    #     if (geometryList[i], geometryList[i+1]) != (graph.nodes[edge[0]]['x'], graph.nodes[edge[0]]['y']):
    #         geometryList = reverse_geometry_list(geometryList)
    #     lengthSegment = tools.distance(geometryList[i],geometryList[i+1],geometryList[i+2],geometryList[i+3])
    #     while loop:
    #         if t > next_time_stamp and time_stamp_index < len(self.speed_time_stamps) - 1:
    #             time_stamp_index += 1
    #             next_time_stamp = self.speed_time_stamps[time_stamp_index][0]
    #             drone_speed = self.speed_time_stamps[time_stamp_index][1]
    #
    #         if lengthSegment >= dist_to_travel:
    #             v = (geometryList[i+2]-x0, geometryList[i+3]-y0)
    #             x0, y0 = dist_to_travel/lengthSegment*v[0] + x0, dist_to_travel/lengthSegment*v[1] + y0
    #             t += dt
    #             self.path_dict_discretized[t] = (x0, y0)
    #             lengthSegment -= dist_to_travel
    #             dist_to_travel = dt*drone_speed
    #             traveled = 0
    #         else:
    #             dist_to_travel -= lengthSegment
    #             traveled += lengthSegment
    #             # TODO a faire au propre
    #             try:
    #                 i += 2
    #                 lengthSegment = tools.distance(geometryList[i],geometryList[i+1],geometryList[i+2],geometryList[i+3])
    #                 x0, y0 = geometryList[i], geometryList[i+1]
    #             except Exception:
    #                 if edgeIndex < len(self.path)-2:
    #                     edgeIndex += 1
    #                     edge = [self.path[edgeIndex], self.path[edgeIndex+1]]
    #                     geometryList = graph.edges[edge]["geometry"]
    #                     i = 0
    #                     x0, y0 = graph.nodes[edge[0]]['x'], graph.nodes[edge[0]]['y']
    #                     if (geometryList[i], geometryList[i+1]) != (x0, y0):
    #                         geometryList = reverse_geometry_list(geometryList.copy())
    #                     lengthSegment = tools.distance(geometryList[i],geometryList[i+1],geometryList[i+2],geometryList[i+3])
    #                     if edge[0] in self.delay:
    #                         c = self.delay[edge[0]]//dt
    #                         for j in range(c):
    #                             t += dt
    #                             self.path_dict_discretized[t] = (x0, y0)
    #                 else:
    #                     loop = False
    #     t += traveled/drone_speed
    #     self.path_dict_discretized[t] = (xEnd, yEnd)


def reverse_geometry_list(listGeo):
    reversedList = []
    for i in range(len(listGeo)-1,0,-2):
        reversedList.append(listGeo[i-1])
        reversedList.append(listGeo[i])
        
    return reversedList


# def new_coords(pathDict, coords, lengthSegment, lengthTraveled, t, dt, i, lengthEdge):
#     '''Finds the new coordinates for the discretization of the path.'''
#     v = (coords[i+2]-coords[i], coords[i+3]-coords[i+1])
#     l0 = tools.distance(coords[i], coords[i+1], coords[i+2], coords[i+3])
#     x0, y0 = coords[i], coords[i+1]
#     offset = lengthTraveled - lengthSegment
#     if l0 != 0:
#         x, y = offset/l0*v[0] + x0, offset/l0*v[1] + y0
#
#         distance_from_previous_node = tools.distance(coords[0], coords[1], x, y)
#
#         if distance_from_previous_node >= lengthEdge:
#             return False, offset, t, x0, y0
#
#         else:
#             t += dt
#             pathDict[t] = (x, y)
#
#             return True, offset, t, x0, y0
#     return False, offset, t, x0, y0
