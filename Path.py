# import tools as tools
import Drone
import tools

over_estimate_turn_factor = 1.2


class Path:
    def __init__(self, hStart, path, drone):
        # Path contain the pth as a list of node
        # if path is None:
        #     path = []
        self.path = path
        self.drone = drone
        # Contains informations on the first and last segment (from and to vertiports)
        self.first_segment = None
        self.last_segment = None
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
        self.turns = []
        # The discretized version of the path with time as key and position x,y as value
        self.path_dict_discretized = {}
        self.delay = {}
        self.dep_time = hStart
        self.arr_time = None
        self.flightTime = 0
        self.flightDistance = 0

    def set_first_segment(self, model):
        """Compute first segment and return time of exit, speed, separation at node and the turn angle"""
        # TODO : return SEP
        dep_edge = invert_dep_edge_if_needed(self.path, self.drone)
        dep_vertiport = self.drone.departure_vertiport
        x_dep, y_dep = float(dep_vertiport[0]), float(dep_vertiport[1])

        # We need to know if we are in the constrained or unconstrained airspace, we use the length of the node
        # to know if it's part of the unconstrained airspace ( == 6)
        if len(self.path[0]) == 6 or dep_edge is None:
            # Then we are in the unconstrained airspace and don't need to worry about turns
            drone_speed_next_node = Drone.speeds_dict["cruise"]
            next_node = self.path[0]
            x_next, y_next = model.graph.nodes[next_node]["x"], model.graph.nodes[next_node]["y"]
            dist_to_next_node = tools.distance(x_dep, y_dep, x_next, y_next)
            sep = return_sep(model, self.drone, Drone.speeds_dict["cruise"], drone_speed_next_node)
            return dist_to_next_node/drone_speed_next_node, drone_speed_next_node, 0, sep
        else:
            # Then this means we are entering in the constrained airspace
            next_edge = (self.path[0], self.path[1])
            # print((dep_edge, next_edge))
            angle = model.graph_dual.edges[(dep_edge, next_edge)]["angle"]
            dist_to_next_node = model.graph.edges[dep_edge]["length"]/2
            # Determine travel time to node :
            drone_speed_next_node = Drone.return_speed_from_angle(angle)
            if drone_speed_next_node == Drone.speeds_dict["cruise"]:
                # If there's no turn
                sep = return_sep(model, self.drone, Drone.speeds_dict["cruise"], drone_speed_next_node)
                return dist_to_next_node/drone_speed_next_node, drone_speed_next_node, 0, sep
            else:
                # If there's a turn
                braking_distance = Drone.return_braking_distance(drone_speed_next_node, Drone.speeds_dict["cruise"])
                if dist_to_next_node > braking_distance:
                    time_to_next_node = Drone.return_accel_time(drone_speed_next_node, Drone.speeds_dict["cruise"]) + (dist_to_next_node - braking_distance)/drone_speed_next_node
                    sep = return_sep(model, self.drone, Drone.speeds_dict["cruise"], drone_speed_next_node)
                    return time_to_next_node, drone_speed_next_node, angle, sep
                else:
                    time_to_next_node = dist_to_next_node/drone_speed_next_node
                    sep = return_sep(model, self.drone, Drone.speeds_dict["cruise"], drone_speed_next_node)
                    return time_to_next_node, drone_speed_next_node, angle, sep

    def set_path(self, new_path, model):
        """Adds a new path to the drone, in terms of waypoints (self.path) and waypoints
                and timestamps (self.pathDict) using the dual graph to set the timestamps with
                the added turning times into account"""
        graph_dual = model.graph_dual
        graph = model.graph
        self.path_dict = {}
        self.path = new_path
        self.turns = [0]
        # Compute time to travel the first segment and the speed at the first node
        self.first_segment = self.set_first_segment(model)
        self.speed_time_stamps[self.dep_time] = Drone.speeds_dict["cruise"]
        t = self.dep_time + self.first_segment[0]
        self.separation_dict[t] = self.first_segment[3]

        self.speed_time_stamps[t] = Drone.speeds_dict["cruise"]
        self.fixed_speed_wpt.append(None)
        self.path_dict[t] = new_path[0]
        # TODO ajouter le premier point dans le sep_dict car maintenant le premier point c'est plus le depart
        # drone_speed = drone.cruise_speed
        speed_at_current_node = Drone.speeds_dict["cruise"]
        # TODO AJOUTER t supplementaire depart depuis milieu edge depart

        for index in range(1, len(new_path)):
            current_edge = (new_path[index-1], new_path[index])
            # length = graph_dual.edges[current_edge_dual]["length"]
            length = graph.edges[current_edge]["length"]
            fixed_speed = None
            middle_value = None

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
            if current_node_is_turn:
                self.turns.append(angle)
            else:
                self.turns.append(0)

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
                    middle_value = (t + travel_time/2, speed_at_current_node)

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
                    middle_value = (t + travel_time / 2, speed_at_current_node)

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
            if middle_value is None:
                self.edge_middle_dict[current_edge] = (t_at_middle, speed_middle)
            else:
                self.edge_middle_dict[current_edge] = (t_at_middle, middle_value)

        # Add last segment
        self.last_segment = self.set_last_segment(model)
        time_to_arr, last_sep, _angle = self.last_segment
        self.separation_dict[t] = last_sep
        self.arr_time = t + time_to_arr
        self.speed_time_stamps[self.arr_time] = Drone.speeds_dict["cruise"]
        self.turns.append(0)

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

    def set_last_segment(self, model):
        """Determine travel time on last segment to arr, and sep at before last node"""
        arr_edge = self.drone.arr_edge
        arr_vertiport = self.drone.arrival_vertiport
        x_arr, y_arr = float(arr_vertiport[0]), float(arr_vertiport[1])
        # Unconstrained or constrained airspace ?
        if arr_edge is None:
            current_node = self.path[-1]
            x_node, y_node = model.graph.nodes[current_node]["x"], model.graph.nodes[current_node]["y"]
            dist_to_arr = tools.distance(x_arr, y_arr, x_node, y_node)
            time_to_arr = dist_to_arr/Drone.speeds_dict["cruise"]
            sep = return_sep(model, self.drone, Drone.speeds_dict["cruise"], Drone.speeds_dict["cruise"])
            return time_to_arr, sep, 0
        else:
            # Is there a turn ?
            last_edge = (self.path[-2], self.path[-1])
            angle = model.graph_dual.edges[last_edge, arr_edge]["angle"]
            dist_to_arr = model.graph.edges[arr_edge]["length"]/2
            # Determine travel time to node :
            drone_speed_last_node = Drone.return_speed_from_angle(angle)
            accel_dist = Drone.return_braking_distance(drone_speed_last_node, Drone.speeds_dict["cruise"])
            if dist_to_arr > accel_dist:
                time_to_arr = Drone.return_accel_time(drone_speed_last_node, Drone.speeds_dict["cruise"])
                time_to_arr += (dist_to_arr - accel_dist) / Drone.speeds_dict["cruise"]
            else:
                time_to_arr = Drone.return_accel_time(drone_speed_last_node, Drone.speeds_dict["cruise"])
            sep = return_sep(model, self.drone, drone_speed_last_node, Drone.speeds_dict["cruise"])
            return time_to_arr, sep, angle

    def flight_time_and_distance(self, graph):
        self.flightDistance = 0
        for i in range(len(self.path)-1):
            self.flightDistance += graph.edges[self.path[i], self.path[i+1]]['length']
        self.flightTime = self.speed_time_stamps[-1][0]
        for node in self.delay:
            self.flightTime += self.delay[node]

    #     self.path_dict_discretized[t] = (xEnd, yEnd)


def reverse_geometry_list(listGeo):
    reversedList = []
    for i in range(len(listGeo)-1,0,-2):
        reversedList.append(listGeo[i-1])
        reversedList.append(listGeo[i])
        
    return reversedList


def return_sep(model, drone, v1, v2):
    protec = model.protection_area
    braking = Drone.return_braking_distance(v1, v2)
    braking_time = Drone.return_accel_time(v1, v2)
    if protec > braking:
        sep = braking_time + (protec - braking) / v2
        return sep, sep
    else:
        # TODO a preciser
        return braking_time, braking_time


def invert_dep_edge_if_needed(new_path, drone):
    dep_edge = drone.dep_edge
    # print(dep_edge, (new_path[0], new_path[1]))
    if drone.dep_edge is not None:
        if drone.dep_edge == (new_path[0], new_path[1]):
            # print("Inverting dep_edge")
            # print(new_path)
            new_path.pop(0)
            # new_dep_edge = (dep_edge[1], dep_edge[0])
            # print(new_dep_edge, (new_path[0], new_path[1]))
            # print("invert and supp")
            return dep_edge
        if drone.dep_edge[0] == new_path[0]:
            new_dep_edge = (dep_edge[1], dep_edge[0])
            # print(new_dep_edge, (new_path[0], new_path[1]))
            # print("inverting")
            return new_dep_edge
    return dep_edge


