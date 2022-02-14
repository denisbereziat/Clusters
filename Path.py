import tools as tools
import Drone


class Path:
    def __init__(self, hStart, path=None):
        # Path contain the pth as a list of node
        if path is None:
            path = []
        self.path = path
        # Previous path is used to store the last path that was taken
        # TODO ajouter un set path pour forcer la mise a jour de previous path aussi
        # TODO passer les attributs en private et forcer l'utilisation des setters et getters
        self.previous_path = []
        # Path dict contain a path with the time as keys and the node as value
        self.path_dict = {}
        self.edge_path = []
        # This list contain the speed and time_stamps, the speed for each time stamp is the speed before reaching it
        # (in between the last one and the current one)
        self.speed_time_stamps = []
        # The discretized version of the path with time as key and position x,y as value
        self.path_dict_discretized = {}
        self.delay = {}
        self.hStart = hStart
        # TODO ça va pas du tout ça il faut supprimer ces attributs et laisser que la methode qui calcule
        self.flightTime = 0
        self.flightDistance = 0

    def set_path(self, new_path, graph, graph_dual, drone):
        """Adds a new path to the drone, in terms of waypoints (self.path) and waypoints
        and timestamps (self.pathDict) using the dual graph to set the timestamps with
        the added turning times into account"""
        self.path_dict = {}
        self.path = new_path
        t = self.hStart
        self.path_dict[t] = new_path[0]
        # TODO implement case where path length = 2
        # drone_speed = drone.cruise_speed
        self.edge_path = []
        for node_index in range(len(new_path)-1):
            self.edge_path.append((new_path[node_index], new_path[node_index+1]))
        for node_index in range(1, len(new_path)-1):
            # print(new_path)
            # To have the speed on the edge we need to know if the last node was a turn and if the next one is
            previous_edge, current_edge, next_edge = None, None, None
            # drone_speed = drone.cruise_speed
            # Edge dual is used to add the pre and post turn cost
            edge_dual = ((new_path[node_index-1], new_path[node_index]),
                         (new_path[node_index], new_path[node_index+1]))
            # Determine travel cost to the node (remove the post turn cost)
            try:
                cost = graph_dual.edges[edge_dual]["length"]
            except:
                print(new_path)
                raise Exception

            # If the edge length is more than turn_distance then we have to take into account the deceleration
            # and the phase where the drone was going at his normal speed
            if node_index > 2:
                previous_edge = (new_path[node_index-2], new_path[node_index-1])
            if node_index > 1:
                current_edge = (new_path[node_index-1], new_path[node_index])
            if node_index + 1 < len(new_path) - 1:
                next_edge = (new_path[node_index], new_path[node_index+1])
            # TODO ajouter le cas du fly by
            # If the last node passed was a turn point
            if current_edge is not None and previous_edge is not None:
                if graph_dual.edges[previous_edge, current_edge]["is_turn"]:
                    angle = graph_dual.edges[previous_edge, current_edge]["angle"]
                    turning_speed = Drone.return_speed_from_angle(angle)
                    # drone_speed = drone.turn_speed
                    # drone_speed = turning_speed
                    t += cost / turning_speed
                # If last node wasn't a turn point, the next one may be one
                elif current_edge is not None and next_edge is not None:
                    if graph_dual.edges[current_edge, next_edge]["is_turn"]:
                        angle = graph_dual.edges[current_edge, next_edge]["angle"]
                        turning_speed = Drone.return_speed_from_angle(angle)
                        braking_distance = Drone.return_braking_distance(Drone.speeds_dict["cruise"], turning_speed)
                        # drone_speed = drone.turn_speed
                        if graph.edges[current_edge]["length"] < braking_distance:
                            t += cost / turning_speed
                        else:
                            t += (graph.edges[current_edge]["length"] - braking_distance) / Drone.speeds_dict["cruise"]
                            # Adding the added time with constant deceleration
                            t += braking_distance / ((Drone.speeds_dict["cruise"] + turning_speed) / 2)
                    else:
                        t += cost / Drone.speeds_dict["cruise"]
                else:
                    t += cost / Drone.speeds_dict["cruise"]
            # If it's the first node and there is no previous node we still need to check the next one
            # Else if the next node is a turning point
            elif current_edge is not None and next_edge is not None:
                if graph_dual.edges[current_edge, next_edge]["is_turn"]:
                    angle = graph_dual.edges[current_edge, next_edge]["angle"]
                    turning_speed = Drone.return_speed_from_angle(angle)
                    braking_distance = Drone.return_braking_distance(Drone.speeds_dict["cruise"], turning_speed)
                    # drone_speed = drone.turn_speed
                    if graph.edges[current_edge]["length"] < braking_distance:
                        t += cost / turning_speed
                    else:
                        t += (graph.edges[current_edge]["length"] - braking_distance) / drone.cruise_speed
                        # Adding the added time with constant deceleration
                        t += braking_distance / ((drone.cruise_speed + turning_speed) / 2)
                else:
                    t += cost / Drone.speeds_dict["cruise"]
            # Else if there is no turning before or after
            else:
                t += cost/Drone.speeds_dict["cruise"]

            self.path_dict[t] = new_path[node_index]
            # Add back the post turn cost
            # t += graph_dual.edges[edge_dual]["post_turn_cost"]/drone_speed
            # self.speed_time_stamps.append([t, drone_speed])
        # Last node :
        try:
            cost = graph.edges[new_path[-2], new_path[-1]]["length"]
        except:
            print(len(new_path))
            raise Exception
        #TODO a verifier
        t += cost/Drone.speeds_dict["cruise"]
        # self.speed_time_stamps.append([t, drone_speed])
        self.path_dict[t] = new_path[-1]
        # drone.path_object = self
        
    def flight_time_and_distance(self, graph, drone):
        self.flightDistance = 0
        for i in range(len(self.path)-1):
            self.flightDistance += graph.edges[self.path[i], self.path[i+1]]['length']
        self.flightTime = self.speed_time_stamps[-1][0]
        for node in self.delay:
            self.flightTime += self.delay[node]

    def discretize_path(self, dt, graph, drone):
        loop = True
        x0, y0 = graph.nodes[drone.dep]['x'], graph.nodes[drone.dep]['y']
        xEnd, yEnd = graph.nodes[drone.arr]['x'], graph.nodes[drone.arr]['y']
        edgeIndex = 0
        edge = [self.path[edgeIndex], self.path[edgeIndex+1]]
        i = 0
        t = self.hStart
        self.path_dict_discretized = {t: (x0, y0)}
        if edge[0] in self.delay:
            c = self.delay[edge[0]]//dt
            for j in range(c):
                t += dt
                self.path_dict_discretized[t] = (x0, y0)

        try:
            drone_speed = self.speed_time_stamps[0][1]
        except Exception:
            print(drone.flight_number)
            print(drone.path_object.path)
            print(drone.path_object.path_dict)
            print(drone.path_object.speed_time_stamps)
            raise Exception
        next_time_stamp = self.speed_time_stamps[0][0]
        time_stamp_index = 0

        dist_to_travel = dt*drone_speed
        traveled = 0
        geometryList = graph.edges[edge]["geometry"]
        if (geometryList[i], geometryList[i+1]) != (graph.nodes[edge[0]]['x'], graph.nodes[edge[0]]['y']):
            geometryList = reverse_geometry_list(geometryList)
        lengthSegment = tools.distance(geometryList[i],geometryList[i+1],geometryList[i+2],geometryList[i+3])
        while loop:
            if t > next_time_stamp and time_stamp_index < len(self.speed_time_stamps) - 1:
                time_stamp_index += 1
                next_time_stamp = self.speed_time_stamps[time_stamp_index][0]
                drone_speed = self.speed_time_stamps[time_stamp_index][1]

            if lengthSegment >= dist_to_travel:
                v = (geometryList[i+2]-x0, geometryList[i+3]-y0) 
                x0, y0 = dist_to_travel/lengthSegment*v[0] + x0, dist_to_travel/lengthSegment*v[1] + y0
                t += dt
                self.path_dict_discretized[t] = (x0, y0)
                lengthSegment -= dist_to_travel
                dist_to_travel = dt*drone_speed
                traveled = 0
            else:
                dist_to_travel -= lengthSegment
                traveled += lengthSegment
                # TODO a faire au propre
                try:
                    i += 2
                    lengthSegment = tools.distance(geometryList[i],geometryList[i+1],geometryList[i+2],geometryList[i+3])
                    x0, y0 = geometryList[i], geometryList[i+1]
                except Exception:
                    if edgeIndex < len(self.path)-2:
                        edgeIndex += 1 
                        edge = [self.path[edgeIndex], self.path[edgeIndex+1]]
                        geometryList = graph.edges[edge]["geometry"]
                        i = 0
                        x0, y0 = graph.nodes[edge[0]]['x'], graph.nodes[edge[0]]['y']
                        if (geometryList[i], geometryList[i+1]) != (x0, y0):
                            geometryList = reverse_geometry_list(geometryList.copy())
                        lengthSegment = tools.distance(geometryList[i],geometryList[i+1],geometryList[i+2],geometryList[i+3])
                        if edge[0] in self.delay:
                            c = self.delay[edge[0]]//dt
                            for j in range(c):
                                t += dt
                                self.path_dict_discretized[t] = (x0, y0)
                    else:
                        loop = False
        t += traveled/drone_speed
        self.path_dict_discretized[t] = (xEnd, yEnd)


def reverse_geometry_list(listGeo):
    reversedList = []
    for i in range(len(listGeo)-1,0,-2):
        reversedList.append(listGeo[i-1])
        reversedList.append(listGeo[i])
        
    return reversedList


def new_coords(pathDict, coords, lengthSegment, lengthTraveled, t, dt, i, lengthEdge):
    '''Finds the new coordinates for the discretization of the path.'''
    v = (coords[i+2]-coords[i], coords[i+3]-coords[i+1])
    l0 = tools.distance(coords[i], coords[i+1], coords[i+2], coords[i+3])
    x0, y0 = coords[i], coords[i+1]
    offset = lengthTraveled - lengthSegment
    if l0 != 0:
        x, y = offset/l0*v[0] + x0, offset/l0*v[1] + y0

        distance_from_previous_node = tools.distance(coords[0], coords[1], x, y)
        
        if distance_from_previous_node >= lengthEdge:
            return False, offset, t, x0, y0
    
        else:
            t += dt
            pathDict[t] = (x, y)
        
            return True, offset, t, x0, y0
    return False, offset, t, x0, y0
