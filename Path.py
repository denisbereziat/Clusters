import math
import Drone
import tools
from Drone import Integrator

over_estimate_turn_factor = 1
worse_hor_time_sep = 0 #initial max value


class Path:
    def __init__(self, hStart, path, drone):
        self.drone = drone
        #Path contains the ordered list of graph nodes (no dep and arr)
        self.path = path
        #departure and arrival time at dep/arr vertiports (horizontal path considered only)
        self.dep_time = hStart
        self.arr_time = None
        # Contains informations on the first and last segment (from and to vertiports)
        self.first_segment = None
        self.last_segment = None
        # Path dict contains a path with the time as keys and the node as value
        # only graph nodes are considered (no dep and arr)
        self.path_dict = {}
        #previous/next_node dicts for all nodes (including dep and arr) provide it's next or previous
        #'nodes' are given as pair of (time, node_id) 
        self.previous_node_dict = dict()
        self.next_node_dict = dict()
        #sep dict contains time as a key and (sep_before, sep_after) the node coresponding to time as value
        #all nodes are included (dep and arr too)
        self.separation_dict = dict()
        #edge_middle dict for each edge that is a key contains (t_middle, (sep_before, sep_after)) as value
        self.edge_middle_dict = dict()
        #ordered list of turn angles len(turns) == len(path)
        self.turns = []
        #list of Integrator objects for each segment of horizontal path
        self.segments = []
        
    def __str__(self):
        res = "Drone: " + self.drone.drone_type + "\n"
        res += str(self.drone.departure_vertiport) + " -- " + str(self.path) + " -- " + str(self.drone.arrival_vertiport) + "\n" 
        res += str(self.path_dict) + "\n"
        for segment in self.segments:
            res += str(segment) + "\n"
        return res
    
    def new_set_path(self, node_list, model):
        """Adds a new path to the drone, in terms of waypoints (self.path) and waypoints
        and timestamps (self.pathDict) using the dual graph to set the timestamps with
        the added turning times into account"""
        graph_dual = model.graph_dual
        graph = model.graph

        #ASSUMPTION: nodel_list contains only graph nodes and doesn't over-pass dep/arr
        #dep_edge, arr_edge = Path.fix_start_end_nodes(node_list, self.drone)
        self.path = node_list
        dep_edge = self.drone.dep_edge
        if dep_edge[0] == self.path[0]:#inverse dep_edge
            dep_edge = (dep_edge[1], dep_edge[0])
        arr_edge = self.drone.arr_edge
        if arr_edge[1] == self.path[-1]:#inverse arr_edge
            arr_edge = (arr_edge[1], arr_edge[0])
        
        dep = self.drone.departure_vertiport
        arr = self.drone.arrival_vertiport
        
        #ASSUMPTION: there is at least one traj point beside dep and arr
        #determine edge of the first segment and it's lenght 
        if not self.drone.is_unconstrained_departure:
            edge1 = dep_edge
            d1 = model.graph.edges[edge1]["length"]/2
        else:
            edge1 = None
            x_dep, y_dep = dep[0], dep[1]
            x_next, y_next = model.graph.nodes[node_list[0]]["x"], model.graph.nodes[node_list[0]]["y"]
            d1 = tools.distance(x_dep, y_dep, x_next, y_next)
        
        #determine edge of the second segment and it's lenght
        edge2, d2 = Path.path_next_segment(0, node_list, arr, arr_edge, not(self.drone.is_unconstrained_arrival), model)

        p = [None, dep, node_list[0]]
        v = [None, 0, self.drone.speeds_dict["cruise"]]
        self.turns.append(0)
        #calculate the real speed at first node in node_list
        #if we are 'entering' in constrained airspace
        if edge2:
            if not(len(edge2[0]) == 6 or len(edge2[1]) == 6): #if edge2 is in constrained
                #we need to compute the angle and coresponding speed
                if edge1:
                    angle = graph_dual.edges[(edge1, edge2)]["angle"]
                else:
                    #compute angle as heading difference of (dep, edge2[0]) and edge2
                    pt1 = model.graph.nodes[edge2[0]]["x"], model.graph.nodes[edge2[0]]["y"]
                    pt2 = model.graph.nodes[edge2[1]]["x"], model.graph.nodes[edge2[1]]["y"]
                    angle = tools.angle_btw_vectors(dep, pt1, pt2)
                v[-1] = Drone.return_speed_from_angle(angle, self.drone)
                self.turns[-1] = angle

        t = self.dep_time
        self.separation_dict[t] = [0, 0]  #initialize next sep_dict key
        current_node = (t, p[1])
        index = 1
        angle0 = 0
        while index < len(node_list):
            angle1 = self.turns[-1]
            #take out the first node and cooresponding speed
            p.pop(0); v.pop(0)
            #append the next node and cooresponding speed
            p.append(node_list[index])
            edge1 = edge2
            edge2, d2_next = Path.path_next_segment(index, node_list, arr, arr_edge, not(self.drone.is_unconstrained_arrival), model)
            #calculate the real speed at node_list[index]
            v.append(self.drone.speeds_dict["cruise"])
            self.turns.append(0)
            if edge2:
                if not(len(edge2[0]) == 6 or len(edge2[1]) == 6): #if edge2 is in constrained
                    #we need to compute the angle and coresponding speed
                    angle = graph_dual.edges[(edge1, edge2)]["angle"]
                    v[-1] = Drone.return_speed_from_angle(angle, self.drone)
                    self.turns[-1] = angle
            
            #integrate segment and save path fields
            t, next_node = self.integrate_segment(t, current_node, p, v, d1, d2, angle0, angle1, model.protection_area, dep, arr)
                
            #set next node
            index += 1
            d1 = d2
            d2 = d2_next
            current_node = next_node
            angle0 = angle1
        
        angle1 = self.turns[-1]
        #take out the first node and cooresponding speed
        p.pop(0); v.pop(0)
        #add the arrival node and cooresponding speed
        p.append(arr)
        v.append(0)
        #integrate segment and save path fields
        t, next_node = self.integrate_segment(t, current_node, p, v, d1, d2, angle0, angle1, model.protection_area, dep, arr)
        
        #treate last segment
        angle0 = angle1
        angle1 = 0
        d1 = d2
        d2 = None
        current_node = next_node
        #take out the first node and cooresponding speed, and don't add anything
        p.pop(0); v.pop(0)
        #integrate last segment and save path fields except path_dict
        t, next_node = self.integrate_segment(t, current_node, p, v, d1, d2, angle0, angle1, model.protection_area, dep, arr, False)
        
        #set Path object arrival time
        self.arr_time = t

    def integrate_segment(self, t, current_node, p, v, d1, d2, angle0, angle1, sep_norm, dep, arr, save_path_dict=True):
        global worse_hor_time_sep #we'll calculate acctual worse horizontal time separation
        #integrate segment between p[0] and p[1]
        integrator = Integrator.integrate(p, v, d1, d2, angle0, angle1, self.drone)
        self.segments.append(integrator)
        
        #set after_sep for current node
        self.separation_dict[t][1] = integrator.time_at_distance_after_begin(sep_norm)
        worse_hor_time_sep = max(worse_hor_time_sep, self.separation_dict[t][1])
        
        t += integrator.end_time()
        #set next node and fill next/previous node dicts
        next_node = (t, p[1])
        self.next_node_dict[current_node] = next_node
        self.previous_node_dict[next_node] = current_node
        
        #add path_dict (no dep/arr)
        if save_path_dict: self.path_dict[t] = p[1]
        
        #initialize and set before_sep for next node
        self.separation_dict[t] = [integrator.time_at_distance_before_end(sep_norm), 0]
        worse_hor_time_sep = max(worse_hor_time_sep, self.separation_dict[t][0])
        
        #initialize and set middle_edge_dict
        #Attention: don't save dep_vert-xxx and xxx-arr_vert, since may cause error later
        if p[0] != dep and p[1] != arr:
            self.edge_middle_dict[p[0], p[1]] = (integrator.middle_time(), (integrator.time_at_distance_before_middle(sep_norm), integrator.time_at_distance_after_middle(sep_norm)))
            worse_hor_time_sep = max(worse_hor_time_sep, self.edge_middle_dict[p[0], p[1]][1][0], self.edge_middle_dict[p[0], p[1]][1][1])
        
        return t, next_node

    def path_next_segment(index, path, arr, arr_edge, arr_in_constraint, model):
        if index < len(path) - 1:
            edge = (path[index], path[index+1])
            d = model.graph.edges[edge]["length"]
        elif arr_in_constraint:
            edge = arr_edge
            d = model.graph.edges[edge]["length"]/2
        else:
            edge = None
            x_curr, y_curr = model.graph.nodes[path[index]]["x"], model.graph.nodes[path[index]]["y"]
            x_arr, y_arr = arr[0], arr[1]
            d = tools.distance(x_curr, y_curr, x_arr, y_arr)
        return edge, d

    def set_first_segment(self, model):
        """Compute first segment and return time of exit, speed, separation at node and the turn angle"""
        # TODO : return SEP
        dep_edge = self.drone.dep_edge
        dep_vertiport = self.drone.departure_vertiport
        x_dep, y_dep = float(dep_vertiport[0]), float(dep_vertiport[1])
        speeds_dict = self.drone.speeds_dict
        # We need to know if we are in the constrained or unconstrained airspace, we use the length of the node
        # to know if it's part of the unconstrained airspace ( == 6)
        if len(self.path[0]) == 6 or self.drone.is_unconstrained_departure:
            # Then we are in the unconstrained airspace and don't need to worry about turns
            drone_speed_next_node = speeds_dict["cruise"]
            next_node = self.path[0]
            x_next, y_next = model.graph.nodes[next_node]["x"], model.graph.nodes[next_node]["y"]
            dist_to_next_node = tools.distance(x_dep, y_dep, x_next, y_next)
            # seg_prm = compute_segment(0, drone_speed_next_node, dist_to_next_node, self.drone)
            v1, v2, v3 = 0, self.drone.speeds_dict["cruise"], self.drone.speeds_dict["cruise"]
            len_next_edge = model.graph.edges[(self.path[0], self.path[1])]["length"]
            sep = return_sep(model, self.drone, v1, v2, v3, dist_to_next_node, len_next_edge)
            accel_dist = Drone.return_braking_distance(speeds_dict["cruise"], 0)
            if accel_dist < dist_to_next_node:
                accel_time = Drone.return_accel_time(speeds_dict["cruise"], 0)
                travel_time = accel_time + (dist_to_next_node - accel_dist)/drone_speed_next_node
            else:
                travel_time = math.sqrt(2 * dist_to_next_node / Drone.accel_max)
            return travel_time, drone_speed_next_node, 0, sep
        else:
            # Then this means we are entering in the constrained airspace
            next_edge = (self.path[0], self.path[1])
            # print((dep_edge, next_edge))
            if (dep_edge, next_edge) in model.graph_dual.edges:
                angle = model.graph_dual.edges[(dep_edge, next_edge)]["angle"]
            else:
                angle = model.graph_dual.edges[((dep_edge[1], dep_edge[0]), next_edge)]["angle"]
            dist_to_next_node = model.graph.edges[dep_edge]["length"]/2
            # Determine travel time to node :
            drone_speed_next_node = Drone.return_speed_from_angle(angle, self.drone)
            if drone_speed_next_node == speeds_dict["cruise"]:
                # If there's no turn
                v1, v2, v3 = 0, self.drone.speeds_dict["cruise"], drone_speed_next_node
                len_next_edge = model.graph.edges[(self.path[0], self.path[1])]["length"]
                sep = return_sep(model, self.drone, v1, v2, v3, dist_to_next_node, len_next_edge)
                return dist_to_next_node/drone_speed_next_node, drone_speed_next_node, 0, sep
            else:
                # If there's a turn
                braking_distance = Drone.return_braking_distance(drone_speed_next_node, speeds_dict["cruise"])
                if dist_to_next_node > braking_distance:
                    time_to_next_node = Drone.return_accel_time(drone_speed_next_node, speeds_dict["cruise"]) + (dist_to_next_node - braking_distance)/drone_speed_next_node
                    v1, v2, v3 = 0, self.drone.speeds_dict["cruise"], drone_speed_next_node
                    len_next_edge = model.graph.edges[(self.path[0], self.path[1])]["length"]
                    sep = return_sep(model, self.drone, v1, v2, v3, dist_to_next_node, len_next_edge)
                    return time_to_next_node, drone_speed_next_node, angle, sep
                else:
                    time_to_next_node = dist_to_next_node/drone_speed_next_node
                    v1, v2, v3 = 0, self.drone.speeds_dict["cruise"], drone_speed_next_node
                    len_next_edge = model.graph.edges[(self.path[0], self.path[1])]["length"]
                    sep = return_sep(model, self.drone, v1, v2, v3, dist_to_next_node, len_next_edge)
                    return time_to_next_node, drone_speed_next_node, angle, sep

    def set_path(self, new_path, model):
        """Adds a new path to the drone, in terms of waypoints (self.path) and waypoints
                and timestamps (self.pathDict) using the dual graph to set the timestamps with
                the added turning times into account"""
        graph_dual = model.graph_dual
        graph = model.graph
        speeds_dict = self.drone.speeds_dict
        self.path_dict = {}
        self.path = new_path
        self.turns = [0]
        # Compute time to travel the first segment and the speed at the first node
        self.first_segment = self.set_first_segment(model)
        #self.speed_time_stamps[self.dep_time] = speeds_dict["cruise"]
        t = self.dep_time + self.first_segment[0]
        self.separation_dict[t] = self.first_segment[3]
        #self.speed_time_stamps[t] = speeds_dict["cruise"]
        #self.fixed_speed_wpt.append(None)
        self.path_dict[t] = new_path[0]
        # TODO ajouter le premier point dans le sep_dict car maintenant le premier point c'est plus le depart
        speed_at_current_node = speeds_dict["cruise"]

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
                speed_at_previous_node = Drone.return_speed_from_angle(angle, self.drone)
                if speed_at_previous_node != speeds_dict["cruise"]:
                    last_node_was_turn = True

            # Check if this node is a turn
            current_node_is_turn = False
            if index < len(new_path) - 1:
                current_edge_dual = ((new_path[index - 1], new_path[index]), (new_path[index], new_path[index + 1]))
                angle = graph_dual.edges[current_edge_dual]["angle"]
                speed_at_current_node = Drone.return_speed_from_angle(angle, self.drone)
                if speed_at_current_node != speeds_dict["cruise"]:
                    current_node_is_turn = True
            if current_node_is_turn:
                self.turns.append(angle)
            else:
                self.turns.append(0)

            # Determine travel time on edge and arrival time at the node along with the
            # 2 separation time (before and after the node)
            # If both nodes were turns
            if last_node_was_turn and current_node_is_turn:
                accel_distance_last_node = Drone.return_braking_distance(speed_at_previous_node, speeds_dict["cruise"])
                accel_distance_current_node = Drone.return_braking_distance(speed_at_current_node, speeds_dict["cruise"])
                # TODO a preciser
                if length > accel_distance_last_node + accel_distance_current_node:
                    travel_time = (Drone.return_accel_time(speed_at_previous_node, speeds_dict["cruise"]) + Drone.return_accel_time(speed_at_current_node, speeds_dict["cruise"])) * over_estimate_turn_factor
                    travel_time += (length - accel_distance_last_node - accel_distance_current_node) / speeds_dict["cruise"]
                else:
                    # If there's not enough space to accelerate fully, then speed will be set to the speed of the coming turn
                    inter_accel_distance = Drone.return_braking_distance(speed_at_previous_node, speed_at_current_node)
                    travel_time = (Drone.return_accel_time(speed_at_current_node, speed_at_previous_node))*over_estimate_turn_factor
                    travel_time += (length - inter_accel_distance) / speed_at_current_node
                    fixed_speed = speed_at_current_node
                    middle_value = (t + travel_time/2, speed_at_current_node)

                if model.protection_area < accel_distance_current_node:
                    t_sep = Drone.return_accel_time(speed_at_current_node, speeds_dict["cruise"])
                else:
                    t_sep = Drone.return_accel_time(speed_at_current_node, speeds_dict["cruise"]) + (model.protection_area - accel_distance_current_node)/speed_at_current_node

            # If the node is  a turn point
            elif current_node_is_turn and not last_node_was_turn:
                accel_distance_current_node = Drone.return_braking_distance(speed_at_current_node, speeds_dict["cruise"])
                if length > accel_distance_current_node:
                    travel_time = (Drone.return_accel_time(speed_at_current_node, speeds_dict["cruise"])) * over_estimate_turn_factor
                    travel_time += (length - accel_distance_current_node)/speeds_dict["cruise"]
                    if model.protection_area < accel_distance_current_node:
                        t_sep = Drone.return_accel_time(speed_at_current_node, speeds_dict["cruise"])
                    else:
                        t_sep = Drone.return_accel_time(speed_at_current_node, speeds_dict["cruise"]) + (model.protection_area - accel_distance_current_node)/speeds_dict["cruise"]
                # If there's not enough distance to brake
                else:
                    # The whole arc is done at v_turn
                    travel_time = Drone.return_accel_time(speed_at_current_node, speeds_dict["cruise"])
                    t_sep = Drone.return_accel_time(speed_at_current_node, speeds_dict["cruise"])
                    middle_value = (t + travel_time / 2, speed_at_current_node)

            # If the last node was a turn but not the coming one
            elif last_node_was_turn and not current_node_is_turn:
                accel_distance_last_node = (Drone.return_braking_distance(speed_at_previous_node, speeds_dict["cruise"])) * over_estimate_turn_factor
                travel_time = Drone.return_accel_time(speed_at_previous_node, speeds_dict["cruise"])
                travel_time += (length - accel_distance_last_node)/speeds_dict["cruise"]
                t_sep = model.protection_area / speeds_dict["cruise"]

            # If neither nodes are turning points
            else:
                travel_time = length / speeds_dict["cruise"]
                t_sep = model.protection_area / speeds_dict["cruise"]

            #self.fixed_speed_wpt.append(fixed_speed)

            previous_t = t
            t += travel_time
            self.path_dict[t] = new_path[index]
            #self.speed_time_stamps[t] = speed_at_current_node
            self.separation_dict[t] = (t_sep, t_sep)
            #self.edge_dict[current_edge] = (previous_t, t)
            t_at_middle = (t+previous_t)/2
            if speed_at_previous_node is not None:
                speed_middle = (speed_at_previous_node + speed_at_current_node)/2
            else:
                speed_middle = speed_at_current_node
            if middle_value is None:
                self.edge_middle_dict[current_edge] = (t_at_middle, speed_middle)
            else:
                self.edge_middle_dict[current_edge] = middle_value

        # Add last segment
        self.last_segment = self.set_last_segment(model)
        time_to_arr, last_sep, _angle = self.last_segment
        self.separation_dict[t] = last_sep
        self.arr_time = t + time_to_arr
        #self.speed_time_stamps[self.arr_time] = speeds_dict["cruise"]
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
        speeds_dict = self.drone.speeds_dict
        len_previous_edge = model.graph.edges[(self.path[-2], self.path[-1])]["length"]
        # Unconstrained or constrained airspace ?
        if self.drone.is_unconstrained_arrival:
            current_node = self.path[-1]
            x_node, y_node = model.graph.nodes[current_node]["x"], model.graph.nodes[current_node]["y"]
            dist_to_arr = tools.distance(x_arr, y_arr, x_node, y_node)
            time_to_arr = dist_to_arr/speeds_dict["cruise"]
            v1, v2, v3 = self.drone.speeds_dict["cruise"], self.drone.speeds_dict["cruise"], 0
            sep = return_sep(model, self.drone, v1, v2, v3, len_previous_edge, dist_to_arr)
            return time_to_arr, sep, 0
        else:
            # Is there a turn ?
            last_edge = (self.path[-2], self.path[-1])
            # print("last ",last_edge)
            # print("arr ",arr_edge)
            # print(arr_edge in model.graph_dual.nodes)
            if (last_edge, arr_edge) not in model.graph_dual.edges :
                arr_edge = (arr_edge[1], arr_edge[0])
            angle = model.graph_dual.edges[last_edge, arr_edge]["angle"]
            dist_to_arr = model.graph.edges[arr_edge]["length"]/2
            # Determine travel time to node :
            drone_speed_last_node = Drone.return_speed_from_angle(angle, self.drone)
            accel_dist = Drone.return_braking_distance(drone_speed_last_node, speeds_dict["cruise"])
            if dist_to_arr > accel_dist:
                time_to_arr = Drone.return_accel_time(drone_speed_last_node, speeds_dict["cruise"])
                time_to_arr += (dist_to_arr - accel_dist) / speeds_dict["cruise"]
            else:
                time_to_arr = Drone.return_accel_time(drone_speed_last_node, speeds_dict["cruise"])
            v1, v2, v3 = self.drone.speeds_dict["cruise"], self.drone.speeds_dict["cruise"], 0
            sep = return_sep(model, self.drone, v1, v2, v3, len_previous_edge, dist_to_arr)
            return time_to_arr, sep, angle


def return_sep(model, drone, v1, v2, v3, l1, l2):
    v_cruise = drone.speeds_dict["cruise"]
    t_sep1, t_sep2 = None, None
    protec = model.protection_area
    # 3 CAS :  protec E : accel, cruise ou braking
    # t_sep1 :
    if v1 == v2 and v2 == v_cruise:
        t_sep1 = protec/v1
    else:
        accel1 = Drone.return_braking_distance(v1, v_cruise)
        braking1 = Drone.return_braking_distance(v2, v_cruise)
        if l1 > protec:
            if l1 > accel1 + braking1:
                # Then there's enough room for acceleration to cruise_speed and braking before the node
                delta_accel = l1 - accel1
                if delta_accel > protec:
                    # Then acceleration is done outside protec area
                    if protec > braking1:
                        # Then the whole braking is done in the protection area
                        t_sep1 = Drone.return_accel_time(v_cruise, v2) + (protec - braking1) / v_cruise
                    else:
                        # Then only part of the braking is in the protection area
                        # Find the t of passage a protec entry_point
                        t_to_protec = return_t_at_d(protec - delta_accel, -Drone.accel_max, v1)
                        v_at_protec = v_cruise - Drone.accel_max * t_to_protec
                        t_sep1 = Drone.return_accel_time(v_at_protec, v2)
                else:
                    # Then acceleration is part of the protec area:
                    t_to_protec = return_t_at_d(l1 - protec, Drone.accel_max, v1)
                    v_at_protec = v1 + Drone.accel_max * t_to_protec
                    t_sep1 = Drone.return_accel_time(v_at_protec, v2)
                    t_sep1 += (protec - Drone.return_braking_distance(v_at_protec, v2) - braking1) / v_cruise
                    t_sep1 += Drone.return_accel_time(v_cruise, v2)
            else:
                # Then there's not enough room to reach cruise_speed
                # TODO Cas particulier
                t_sep1 = protec / drone.speeds_dict["turn2"]
        else:
            # TODO Cas particulier
            t_sep1 = protec / drone.speeds_dict["turn2"]
    # t_sep2 :
    if v2 == v3 and v3 == v_cruise:
        t_sep2 = protec / v2
    else:
        accel2 = Drone.return_braking_distance(v2, v_cruise)
        braking2 = Drone.return_braking_distance(v3, v_cruise)
        if l2 > protec:
            if protec < braking2:
                # Then braking finish after the protec area
                t_sep2 = return_t_at_d(protec, Drone.accel_max, v1)
            elif braking2 < protec < (l2 - accel2):
                # Then braking occurs in constant speed segment
                t_sep2 = Drone.return_accel_time(v2, v_cruise)
                t_sep2 += (protec - accel2) / v_cruise
            else:
                # Then braking occurs in the deceleration segment
                t_sep2 = Drone.return_accel_time(v2, v_cruise)
                t_sep2 += (l2 - accel2) / v_cruise
                t_sep2 += return_t_at_d(protec - (l2 - braking2), Drone.accel_max, v1)
        else:
            # TODO cas particulier
            t_sep2 = protec / drone.speeds_dict["turn2"]
    return t_sep1, t_sep2


def return_t_at_d(dist, a, v0):
    t1 = (-v0 + math.sqrt(v0 ** 2 + 2 * a * dist)) / (2 * a)
    if t1 > 0:
        return t1
    else:
        return (-v0 - math.sqrt(v0 ** 2 + 2 * a * dist)) / (2 * a)


def compute_segment(v_prev, v_next, length, drone):

    t0, t1, t2, t3 = 0, None, None, None
    v0, v1, v2, v3 = v_prev, None, None, None
    d0, d1, d2, d3 = 0, None, None, length

    cruise = drone.speeds_dict["cruise"]
    accel_d = Drone.return_braking_distance(cruise, v_prev)
    brake_d = Drone.return_braking_distance(cruise, v_next)
    # + IF V1 sup ou inf a V2
    if length < brake_d or length < accel_d:
        if v_next < v_prev:
            t1, t2 = t0, t0
            v1, v2 = v_prev, v_prev
            d1, d2 = d0, d0
            t3 = return_t_at_d(length, -Drone.accel_max, v_prev)
            v3 = v_prev + Drone.accel_max * t3
            d3 = Drone.return_braking_distance(v_prev, v3)
        if v_next >= v_prev:
            t1 = return_t_at_d(length, Drone.accel_max, v_prev)
            v1 = v_prev + Drone.accel_max * t1
            d1 = Drone.return_braking_distance(v_prev, v1)
            t2, t3 = t1, t1
            v2, v3 = v1, v1
            d2, d3 = d1, d1
    elif length < accel_d + brake_d:
        # Then the drone can't reach it's max speed on this segment.
        v_inter = math.sqrt(length * Drone.accel_max + v_prev ** 2 + v_next ** 2)
        v1, v2 = v_inter
        t_inter = abs(v_inter-v_prev) / Drone.accel_max
        t1, t2 = t_inter
        d_inter = Drone.return_braking_distance(v_inter, v0)
        d1, d2 = d_inter
        t3 = t1 + abs(v_inter-v_next) / Drone.accel_max
        d3 = d_inter + Drone.return_braking_distance(v_inter, v_next)
        v3 = v_next
    else:
        t1 = Drone.return_accel_time(v_next, cruise)
        d1 = Drone.return_braking_distance(v_next, cruise)
        d2 = length - Drone.return_braking_distance(cruise, v_prev)
        t2 = t1 + (d2 - d1) / cruise
        t3 = t2 + Drone.return_braking_distance(cruise, v_next)
        v1, v2, v3 = cruise, cruise, v_next

    return [t0, t1, t2, t3], [v0, v1, v2, v3, d0], [d0, d1, d2, d3]
