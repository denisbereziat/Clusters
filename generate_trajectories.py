"""Generate trajectories to be used in the optim model"""
import math
import tools
import Astar2 as a2
import Path
import pyproj
import matplotlib.pyplot as plt
import itertools
import Drone
from Drone import VerticalIntegrator

verbose = False


def generate_trajectories(model, graph, raw_graph, graph_dual, geofence_time_intervals, current_param=None, number_of_trajectories=None):
    """Generate alternative trajectories for all drones in the model
    OUTPUT :
    drone_trajectories_dict[drone_fn][traj_id] = [path_object, total_time]
    trajectories_to_path[traj_id] = path
    trajectories_to_fn[traj_id] = drone_fn
    trajectories_to_duration[traj_id] = total_time
    fn_order = list of flight_number of the list of model.droneList
    """

    if current_param is not None:
        drone_trajectories_dict, trajectories_to_fn, trajectories_to_duration, trajectories_to_path, fn_order = current_param
    else:
        drone_trajectories_dict, trajectories_to_fn, trajectories_to_duration, trajectories_to_path, fn_order = dict(), dict(), dict(), dict(), []
    if number_of_trajectories is None:
        nb_alternative_traj = 5
    else:
        nb_alternative_traj = number_of_trajectories

    generated_trajectories = dict()
    # Generate alternate trajectories for all drones.
    # If the drone doesn't already have trajectories generated we generate some
    for drone in model.droneList:
        if drone.flight_number not in drone_trajectories_dict:
            generated_trajectories[drone.flight_number] = generate_parallel_trajectories(drone, model, 6, 500, nb_alternative_traj, geofence_time_intervals)

    # Compute total time for each trajectory
    fn_order = []
    for drone in model.droneList:
        fn_order.append(drone.flight_number)
    if current_param is None:
        traj_id = 0  # used to fill the traj_id to traj dict
    else:
        traj_id = max(trajectories_to_fn.keys()) + 1
    # print("Time and path_object")
    for drone_fn in generated_trajectories:
        for traj in generated_trajectories[drone_fn]:
            drone = return_drone_from_flight_number(model, drone_fn)
            path = Path.Path(drone.dep_time, [], drone)
            path.new_set_path(traj, model) #!!!we may directly call it from constructor
            # print("PATH :", path.path_dict)
            # path.set_path(traj, model)
            # print("OLDPATH :", path.path_dict)
            total_time = path.arr_time - path.dep_time
            trajectories_to_path[traj_id] = path
            trajectories_to_fn[traj_id] = drone_fn
            trajectories_to_duration[traj_id] = total_time
            if drone_fn in drone_trajectories_dict:
                drone_trajectories_dict[drone_fn][traj_id] = [path, total_time]
            else:
                drone_trajectories_dict[drone_fn] = dict()
                drone_trajectories_dict[drone_fn][traj_id] = [path, total_time]
            traj_id += 1
    return drone_trajectories_dict, trajectories_to_fn, trajectories_to_duration, trajectories_to_path, fn_order


def save_hor_intersection(trajectories_to_path, k1, k2, multiPoints, multiPointsStatus, horizontal_shared_nodes_list):
    #check what is situation and save intersection(s)
    if verbose:
        print("Connected :", multiPoints)
    if multiPointsStatus == 1: #this could be verified with len of multiPoints
        #save independent intersecting point
        t1 = multiPoints[0][1]
        t2 = multiPoints[0][2]
        conflict = (k1, k2, t1, t2, max(trajectories_to_path[k1].separation_dict[t1][1], trajectories_to_path[k2].separation_dict[t2][0]), max(trajectories_to_path[k2].separation_dict[t2][1], trajectories_to_path[k1].separation_dict[t1][0]))
    else:
        first_node = multiPoints[0]
        last_node = multiPoints[-1]

        t_k1_first = first_node[1]
        t_k1_last = last_node[1]
        
        t_k2_first = first_node[2]
        t_k2_last = last_node[2]
        
        sep12_first = max(trajectories_to_path[k1].separation_dict[t_k1_first][1], trajectories_to_path[k2].separation_dict[t_k2_first][0])
        sep12_last = max(trajectories_to_path[k1].separation_dict[t_k1_last][1], trajectories_to_path[k2].separation_dict[t_k2_last][0])

        sep21_first = max(trajectories_to_path[k2].separation_dict[t_k2_first][1], trajectories_to_path[k1].separation_dict[t_k1_first][0])
        sep21_last = max(trajectories_to_path[k2].separation_dict[t_k2_last][1], trajectories_to_path[k1].separation_dict[t_k1_last][0])
            
        if multiPointsStatus == 2:
            #they are consecutive, take fist and last and create single intersection condition
            cut_k2 = (t_k1_last - t_k1_first) - (t_k2_last - t_k2_first)
            delta_sep12 = sep12_last - sep12_first

            new_sep12_first = sep12_first + max(0, min(cut_k2, cut_k2 + delta_sep12)) + max(0, min(delta_sep12, delta_sep12 + cut_k2))

            cut_k1 = (t_k2_last - t_k2_first) - (t_k1_last - t_k1_first)
            delta_sep21 = sep21_last - sep21_first

            new_sep21_first = sep21_first + max(0, min(cut_k1, cut_k1 + delta_sep21)) + max(0, min(delta_sep21, delta_sep21 + cut_k1))
            
        elif multiPointsStatus == 3:
            #they are opposite
            new_sep12_first = (t_k1_last - t_k1_first) + (t_k2_first - t_k2_last) + sep12_last
            new_sep21_first = sep21_first
        conflict = (k1, k2, t_k1_first, t_k2_first, new_sep12_first, new_sep21_first)
                
    horizontal_shared_nodes_list.append(conflict)
    if verbose:
        print("conflict MultipointStatus", multiPointsStatus, " : ", conflict)


def generate_vert_hor_intersections(vert_intersection_grid, vert_hor_intersection_grid, delta_time, intersection_list, trajectories_to_fn_dict, model):
    #final object in vert_intersection_grid and vert_hor_intersection_grid are
    #triplets (traj_id, time_at_intersection, (hor_time_separation_before, hor_time_separation_before))
    for dep_edge in vert_intersection_grid:
        for time_id in vert_intersection_grid[dep_edge]:
            if verbose:
                print('Current vertical ', vert_intersection_grid[dep_edge][time_id])
            if dep_edge in vert_hor_intersection_grid: #if no hor drones no intersections
                potential_intersections = []
                if time_id in vert_hor_intersection_grid[dep_edge]:
                    if verbose:
                        print('Current hor ', vert_hor_intersection_grid[dep_edge][time_id])
                    #add interaction of current clmb and hor
                    potential_intersections.extend(itertools.product(vert_intersection_grid[dep_edge][time_id], vert_hor_intersection_grid[dep_edge][time_id]))
                    #add interaction of next clmb and current hor
                    if time_id+1 in vert_intersection_grid[dep_edge]:
                        if verbose:
                            print('Next vertical ', vert_intersection_grid[dep_edge][time_id+1])
                        potential_intersections.extend(itertools.product(vert_intersection_grid[dep_edge][time_id+1], vert_hor_intersection_grid[dep_edge][time_id]))
                
                #add interaction of current clmb and next hor
                if time_id+1 in vert_hor_intersection_grid[dep_edge]:
                    if verbose:
                        print('Next hor ', vert_hor_intersection_grid[dep_edge][time_id+1])
                    potential_intersections.extend(itertools.product(vert_intersection_grid[dep_edge][time_id],vert_hor_intersection_grid[dep_edge][time_id+1]))
                    
                for pair in potential_intersections:
                    if verbose:
                        print(pair)
                    if trajectories_to_fn_dict[pair[0][0]] != trajectories_to_fn_dict[pair[1][0]]:
                        if abs(pair[0][1] - pair[1][1]) <= delta_time: #further fillter per actual separation
                            #create intersecting point
                            conflict = (pair[0][0], pair[1][0], pair[0][1], pair[1][1], max(pair[0][2][1], pair[1][2][0]), max(pair[1][2][1], pair[0][2][0]))
                            if verbose:
                                print("It's an conflict ", conflict)
                            intersection_list.append(conflict)


def generate_vertiport_intersections(vert_intersection_grid, delta_time, intersection_list, temps_sep_vertiport, trajectories_to_fn_dict):
    for vertiport in vert_intersection_grid:
        for time_id in vert_intersection_grid[vertiport]:
            if verbose:
                print("Current operations ", vert_intersection_grid[vertiport][time_id])
            potential_intersections = list(itertools.combinations(vert_intersection_grid[vertiport][time_id], 2))
            if time_id+1 in vert_intersection_grid[vertiport]:
                if verbose:
                    print("Next operations ", vert_intersection_grid[vertiport][time_id+1])
                potential_intersections.extend(itertools.product(vert_intersection_grid[vertiport][time_id], vert_intersection_grid[vertiport][time_id+1]))
                    
            for pair in potential_intersections:
                if verbose:
                    print(pair)
                if trajectories_to_fn_dict[pair[0][0]] != trajectories_to_fn_dict[pair[1][0]]:
                    if abs(pair[0][1] - pair[1][1]) <= delta_time: #further fillter per actual separation
                        #create intersecting point
                        conflict = (pair[0][0], pair[1][0], pair[0][1], pair[1][1], temps_sep_vertiport, temps_sep_vertiport)
                        if verbose:
                            print("It's an conflict ", conflict)
                        intersection_list.append(conflict)


def generate_intersection_points(drone_trajectories_dict, trajectories_to_fn_dict, trajectories_to_path, model, graph, graph_dual, raw_graph):
    """Generate intersection points between given trajectories"""

    max_delta_fl = model.nb_FL - 1
    horizontal_shared_nodes_list = []
    climb_horiz_list = []
    descent_horiz_list = []
    climb_climb_list = []
    descent_descent_list = []
    
    #we take into account middle hor turning speed
    #worse_hor_time_sep = model.protection_area / Drone.speeds_dict_model1["turn2"]
    worse_hor_time_sep = Path.worse_hor_time_sep
    #we take into account acceleration (or deceleration) and climb at max speed for the rest
    #use when drone that is climbing/descending is leveling up at next flight level
    worse_vert_time_sep = VerticalIntegrator.integrate(model.FL_sep, 0, Drone.vertical_speed).time_at_distance_before_end(model.vertical_protection)
    #we take into account both acceleration and deceleration and climb at max speed for the rest
    #use when drone is climbing only one level
    worse_vert_time_sep_onelevel = VerticalIntegrator.integrate(model.FL_sep).time_at_distance_before_end(model.vertical_protection)
    
    '''Intersection dicts has key=interescting_point_id (e.g. node, dep, arr, or edge id) 
    and value that is another dict whose key is time_id and value is list of trajectories 
    (plus additional elements that are further needed e.g. time over a point, speeds,etc.)
    * time_id is calculated as int(time_over_node/delta_time)
    * delta_time is used to filter the trajectories and it is calculated such that  
    if trajs time separation is greater than this value then they cann't be in interaction
    * delta_time is calculated as worse time shift that may yield lost of separation in distance
    * Hence, it is given by a sum of max_delay and max sep in time, plus any addition time shift,
    such as level distances, etc. (this depends on the type of intersection)
    Atention!!! always use the same norm to filter and to indentify intersection'''
    # horizontal-horizontal
    delta_time_horizontal = math.ceil(model.delay_max + worse_hor_time_sep)
    horizontal_intersection_grid = {}
    # vertical-horizontal
    delta_time_clmb_hor = math.ceil(model.delay_max + Drone.vertical_speed / (2 * Drone.vertical_accel) + worse_hor_time_sep)
    clmb_intersection_grid = {}
    clmb_hor_intersection_grid = {}
    delta_time_desc_hor = math.ceil(model.delay_max + 2 * max_delta_fl * model.FL_sep / Drone.vertical_speed +
                         Drone.vertical_speed / (2 * Drone.vertical_accel) + worse_hor_time_sep)
    desc_intersection_grid = {}
    desc_hor_intersection_grid = {}
    # vertical-vertical
    delta_time_dep = math.ceil(model.delay_max + model.temps_sep_vertiport)
    dep_intersection_grid = {}
    delta_time_arr = math.ceil(model.delay_max + 2 * max_delta_fl * model.FL_sep / Drone.vertical_speed + model.temps_sep_vertiport)
    arr_intersection_grid = {}
    '''Loop over all drones, trajectories and their nodes and fill the grids 
    by adding concerned trajectories and respective passage time over intersecting point (and any other required data)
    * horizontal_intersection_grid indexed by nodes and normalized time
    * clmb_hor_intersection_grid and desc_hor_intersection_grid indexed by edge_id and normalized time
    (edge_id is the one in which middle the vertiport is located)
    * dep_intersection_grid and arr_intersection_grid indexed by vertiport_id and normalized time'''
    for drone in model.droneList:
        # initialize clmb_intersection_grid and clmb_hor_intersection_grid
        dep_edge = drone.dep_edge
        if not(drone.is_unconstrained_departure) and dep_edge[0] > dep_edge[1]:
            # order only dep_edge that are graph edges i.e. depart from middle of the street
            dep_edge = (dep_edge[1], dep_edge[0])
        if not dep_edge in clmb_intersection_grid:
            clmb_intersection_grid[dep_edge] = {}
        if not dep_edge in clmb_hor_intersection_grid:
            clmb_hor_intersection_grid[dep_edge] = {}
        # initialize desc_intersection_grid and desc_hor_intersection_grid
        arr_edge = drone.arr_edge
        if not(drone.is_unconstrained_arrival) and arr_edge[0] > arr_edge[1]:
            # order only arr_edge that are graph edges i.e. arrival at middle of the street
            arr_edge = (arr_edge[1], arr_edge[0])
        if not arr_edge in desc_intersection_grid:
            desc_intersection_grid[arr_edge] = {}
        if not arr_edge in desc_hor_intersection_grid:
            desc_hor_intersection_grid[arr_edge] = {}
        # initialize dep_intersection_grid
        dep = drone.departure_vertiport
        if not dep in dep_intersection_grid:
            dep_intersection_grid[dep] = {}
        # initialize arr_intersection_grid
        arr = drone.arrival_vertiport
        if not arr in arr_intersection_grid:
            arr_intersection_grid[arr] = {}
        # initialize horizontal_intersection_grid with dep and arr
        if not dep in horizontal_intersection_grid:
            horizontal_intersection_grid[dep] = {}
        if not arr in horizontal_intersection_grid:
            horizontal_intersection_grid[arr] = {}

        drone_fn = drone.flight_number
        for traj_id in drone_trajectories_dict[drone_fn]:
            traj_path = drone_trajectories_dict[drone_fn][traj_id][0]
            # fill clmb_intersection_grid and clmb_hor_intersection_grid
            time_id = traj_path.dep_time // delta_time_clmb_hor
            if not time_id in clmb_intersection_grid[dep_edge]:
                clmb_intersection_grid[dep_edge][time_id] = []
            clmb_intersection_grid[dep_edge][time_id].append((traj_id, traj_path.dep_time, (worse_vert_time_sep, worse_vert_time_sep)))
            if not time_id in clmb_hor_intersection_grid[dep_edge]:
                clmb_hor_intersection_grid[dep_edge][time_id] = []
            clmb_hor_intersection_grid[dep_edge][time_id].append((traj_id, traj_path.dep_time, (worse_vert_time_sep_onelevel, traj_path.separation_dict[traj_path.dep_time][1])))
            # fill desc_intersection_grid and desc_hor_intersection_grid
            time_id = traj_path.arr_time // delta_time_desc_hor
            if not time_id in desc_intersection_grid[arr_edge]:
                desc_intersection_grid[arr_edge][time_id] = []
            desc_intersection_grid[arr_edge][time_id].append((traj_id, traj_path.arr_time, (worse_vert_time_sep, worse_vert_time_sep)))
            if not time_id in desc_hor_intersection_grid[arr_edge]:
                desc_hor_intersection_grid[arr_edge][time_id] = []
            desc_hor_intersection_grid[arr_edge][time_id].append((traj_id, traj_path.arr_time, (traj_path.separation_dict[traj_path.arr_time][0], worse_vert_time_sep_onelevel)))
            # fill dep_intersection_grid
            time_id = traj_path.dep_time // delta_time_dep  # integer division
            if not time_id in dep_intersection_grid[dep]:
                dep_intersection_grid[dep][time_id] = []
            dep_intersection_grid[dep][time_id].append((traj_id, traj_path.dep_time))
            # fill arr_intersection_grid
            time_id = traj_path.arr_time // delta_time_arr  # integer division
            if not time_id in arr_intersection_grid[arr]:
                arr_intersection_grid[arr][time_id] = []
            arr_intersection_grid[arr][time_id].append((traj_id, traj_path.arr_time))
            # fill horizontal_intersection_grid with dep and arr
            time_id = traj_path.dep_time // delta_time_horizontal  # integer division
            if not time_id in horizontal_intersection_grid[dep]:
                horizontal_intersection_grid[dep][time_id] = []
            horizontal_intersection_grid[dep][time_id].append((traj_id, traj_path.dep_time))
            time_id = traj_path.arr_time // delta_time_horizontal  # integer division
            if not time_id in horizontal_intersection_grid[arr]:
                horizontal_intersection_grid[arr][time_id] = []
            horizontal_intersection_grid[arr][time_id].append((traj_id, traj_path.arr_time))
            

            for t, node_id in traj_path.path_dict.items():
                # initialize horizontal_intersection_grid
                time_id = t // delta_time_horizontal  # integer division
                if not node_id in horizontal_intersection_grid:
                    horizontal_intersection_grid[node_id] = {}
                if not time_id in horizontal_intersection_grid[node_id]:
                    horizontal_intersection_grid[node_id][time_id] = []
                # fill horizontal_intersection_grid
                horizontal_intersection_grid[node_id][time_id].append((traj_id, t))

            # We could avoid this second loop if path_dict is already ordered by time and then edges are created on the fly
            for edge_id, (t, sep) in traj_path.edge_middle_dict.items():
                if edge_id[0] > edge_id[1]:
                    edge_id = (edge_id[1], edge_id[0])
                # initialize clmb_hor_intersection_grid
                time_id = t // delta_time_clmb_hor
                if not edge_id in clmb_hor_intersection_grid:
                    clmb_hor_intersection_grid[edge_id] = {}
                if not time_id in clmb_hor_intersection_grid[edge_id]:
                    clmb_hor_intersection_grid[edge_id][time_id] = []
                # fill clmb_hor_intersection_grid
                clmb_hor_intersection_grid[edge_id][time_id].append((traj_id, t, sep))

                # initialize desc_hor_intersection_grid
                time_id = t // delta_time_desc_hor
                if not edge_id in desc_hor_intersection_grid:
                    desc_hor_intersection_grid[edge_id] = {}
                if not time_id in desc_hor_intersection_grid[edge_id]:
                    desc_hor_intersection_grid[edge_id][time_id] = []
                # fill desc_hor_intersection_grid
                desc_hor_intersection_grid[edge_id][time_id].append((traj_id, t, sep))

    '''DETECTION OF HORIZONTAL INTERSECTION POINTS
    Now it is simply necessary to loop over the grid and for every node_id and every time_id 
    then verify all trajectory pairs in this cell and neighbouring ones
    i.e. one time_id before and after are potential interactions
    * If we want to avoid to conservative consideration of intersections, we need to filter further 
    all pairs by the actual time separation whether it is less than delta_time_horizontal
    To avoid duplicates, for every time_id we check all combinations of:
    * trajs in time_id
    * trajs in time_id vs time_id+1
    
    Intersection points could be imediatelly created for every filtered pair,
    but to avoid multiple intersection points (in the case when two trajs share part of their routes)
    we first store all intersecting point for every pair of trajs and apply additional filters.
    So let's buid a new dictionary that will keep all intersecting points for a pair of trajs
    with passage time of first and second trajectory over the points'''
    horizontal_intersections = {}
    for node_id in horizontal_intersection_grid:
        for time_id in horizontal_intersection_grid[node_id]:

            potential_intersections = list(itertools.combinations(horizontal_intersection_grid[node_id][time_id], 2))
            if time_id + 1 in horizontal_intersection_grid[node_id]:
                potential_intersections.extend(itertools.product(horizontal_intersection_grid[node_id][time_id],
                                                                 horizontal_intersection_grid[node_id][time_id + 1]))

            for pair in potential_intersections:
                if trajectories_to_fn_dict[pair[0][0]] != trajectories_to_fn_dict[pair[1][0]]:  # not the same flight intention i.e. drone
                    if abs(pair[0][1] - pair[1][1]) <= delta_time_horizontal:  # further fillterig per actual separation
                        key = (pair[0][0], pair[1][0])
                        val = (node_id, pair[0][1], pair[1][1])
                        if pair[0][0] > pair[1][0]:
                            key = (pair[1][0], pair[0][0])
                            val = (node_id, pair[1][1], pair[0][1])

                        if not key in horizontal_intersections:
                            horizontal_intersections[key] = []
                        horizontal_intersections[key].append(val)

    '''now for every pair of trajectories remove multiple points (that could be consecutive or opposite)
    and save the intersection point(s)'''
    for (k1, k2), points in horizontal_intersections.items():
        if verbose:
            print("k1 k2")
            print(k1, k2)

        if len(points) == 1:
            #this is just simple intersection
            #save intersecting point and continue
            if verbose:
                print("points initial")
                print(points)
            save_hor_intersection(trajectories_to_path, k1, k2, points, 1, horizontal_shared_nodes_list)
        else:
            # there are multiple points, but they could be still: independent or
            # part of the consecutive or opossite portion of traj
            # detect the case, reduce points if needed and save results

            # sort the points by passage time of first traj
            points.sort(key=lambda x: x[1])
            # print("points initial")
            # print(points)

            multiPoints = [points[0]]
            multiPointsStatus = 1  # 1 if independant, 2 if consecutive, 3 if opposite
            point_index = 1
            while point_index < len(points):
                if (multiPoints[-1][1], multiPoints[-1][0]) in trajectories_to_path[k1].next_node_dict:
                    if trajectories_to_path[k1].next_node_dict[(multiPoints[-1][1], multiPoints[-1][0])] == (
                            points[point_index][1], points[point_index][0]):
                        # Potentially there is multiplication
                        if (multiPointsStatus == 1 or multiPointsStatus == 2) and (multiPoints[-1][2], multiPoints[-1][0]) in trajectories_to_path[k2].next_node_dict:
                            if trajectories_to_path[k2].next_node_dict[(multiPoints[-1][2], multiPoints[-1][0])] == (
                                    points[point_index][2], points[point_index][0]):
                                # consecutive multiple points
                                multiPoints.append(points[point_index])
                                multiPointsStatus = 2
                                point_index += 1
                                continue
                        if (multiPointsStatus == 1 or multiPointsStatus == 3) and (multiPoints[-1][2], multiPoints[-1][0]) in trajectories_to_path[k2].previous_node_dict:
                            if trajectories_to_path[k2].previous_node_dict[(multiPoints[-1][2], multiPoints[-1][0])] == (
                                    points[point_index][2], points[point_index][0]):
                                # opposite multiple points
                                multiPoints.append(points[point_index])
                                multiPointsStatus = 3
                                point_index += 1
                                continue

                #multi point chain is broken
                #save intersection(s)
                save_hor_intersection(trajectories_to_path, k1, k2, multiPoints, multiPointsStatus, horizontal_shared_nodes_list)

                #reinitialise multiPoints and multiPointsStatus
                multiPoints = [points[point_index]]
                multiPointsStatus = 1
                point_index += 1
            
            #make last saving
            save_hor_intersection(trajectories_to_path, k1, k2, multiPoints, multiPointsStatus, horizontal_shared_nodes_list)
            
    
    '''DETECTION OF VERT_HOR INTERSECTION POINTS
    Loop over the vertical greed (containing all clmb and desc trajs) and for every edge_id and every time_id 
    verify if vertical trajs are in interaction with horizontal in the neighbourhood.
    To avoid duplicates for every time_id we check all combinations of:
    * vert[time_id] vs hor[time_id]
    * vert[time_id] vs hor[time_id+1]
    * vert[time_id+1] vs hor[time_id]
    The principle is the same for clmb_hor_intersection_grid and desc_hor_intersection_grid just time norm is different'''
    #DETECTION OF CLIMB HORIZONTAL INTERSECTION POINTS
    generate_vert_hor_intersections(clmb_intersection_grid, clmb_hor_intersection_grid, delta_time_clmb_hor, climb_horiz_list, trajectories_to_fn_dict, model)
    #DETECTION OF DESCENT HORIZONTAL INTERSECTION POINTS
    generate_vert_hor_intersections(desc_intersection_grid, desc_hor_intersection_grid, delta_time_desc_hor, descent_horiz_list, trajectories_to_fn_dict, model)
    
    '''DETECTION OF VERTICAL INTERSECTION POINTS
    Loop over the dep_intersection_grid and for every dep point and every time_id 
    verify all pairs of trajectories in the neighbourhood.
    To avoid duplicates for every time_id we check all combinations:
    * of deps[time_id]
    * between deps[time_id] vs deps[time_id+1]
    The principle is the same for arr_intersection_grid just time norm is different.'''
    #DETECTION OF DEP INTERSECTION POINTS
    generate_vertiport_intersections(dep_intersection_grid, delta_time_dep, climb_climb_list, model.temps_sep_vertiport, trajectories_to_fn_dict)
    #DETECTION OF ARR INTERSECTION POINTS
    generate_vertiport_intersections(arr_intersection_grid, delta_time_arr, descent_descent_list, model.temps_sep_vertiport, trajectories_to_fn_dict)

    return horizontal_shared_nodes_list, climb_horiz_list, descent_horiz_list, climb_climb_list, descent_descent_list


def generate_interaction(drone_trajectories_dict, trajectories_to_fn_dict, trajectories_to_path, model, graph, graph_dual, raw_graph):
    """Determine whether there is interaction between given trajectories"""

    #we take into account middle hor turning speed
    #worse_hor_time_sep = model.protection_area / Drone.speeds_dict_model1["turn2"]
    worse_hor_time_sep = Path.worse_hor_time_sep
    
    '''Intersection dicts has key=interescting_point_id and value that is another dict 
    whose key is time_id and value is list of trajectories 
    * time_id is calculated as int(time_over_node/delta_time)
    * delta_time is used to filter the trajectories and it is calculated such that  
    if trajs time separation is greater than this value then they cann't be in interaction
    * delta_time is calculated as worse time shift that may yield lost of separation in distance
    * Hence, it is given by a sum of max_delay and max sep in time, plus any addition time shift,
    such as level distances, etc. (this depends on the type of intersection)
    Atention!!! always use the same norm to filter and to indentify intersection'''
    # horizontal-horizontal
    delta_time_horizontal = math.ceil(model.delay_max + worse_hor_time_sep)
    horizontal_intersection_grid = {}
    '''Loop over all drones, trajectories and their nodes and fill the grids 
    by adding concerned trajectories and respective passage time over intersecting point (and any other required data)
    * horizontal_intersection_grid indexed by nodes and normalized time'''
    for drone in model.droneList:
        # initialize horizontal_intersection_grid with dep and arr
        if not dep in horizontal_intersection_grid:
            horizontal_intersection_grid[dep] = {}
        if not arr in horizontal_intersection_grid:
            horizontal_intersection_grid[arr] = {}

        drone_fn = drone.flight_number
        for traj_id in drone_trajectories_dict[drone_fn]:
            traj_path = drone_trajectories_dict[drone_fn][traj_id][0]
            # fill horizontal_intersection_grid with dep and arr
            time_id = traj_path.dep_time // delta_time_horizontal  # integer division
            if not time_id in horizontal_intersection_grid[dep]:
                horizontal_intersection_grid[dep][time_id] = []
            horizontal_intersection_grid[dep][time_id].append((drone_fn, traj_path.dep_time))
            time_id = traj_path.arr_time // delta_time_horizontal  # integer division
            if not time_id in horizontal_intersection_grid[arr]:
                horizontal_intersection_grid[arr][time_id] = []
            horizontal_intersection_grid[arr][time_id].append((drone_fn, traj_path.arr_time))
            

            for t, node_id in traj_path.path_dict.items():
                # initialize horizontal_intersection_grid
                time_id = t // delta_time_horizontal  # integer division
                if not node_id in horizontal_intersection_grid:
                    horizontal_intersection_grid[node_id] = {}
                if not time_id in horizontal_intersection_grid[node_id]:
                    horizontal_intersection_grid[node_id][time_id] = []
                # fill horizontal_intersection_grid
                horizontal_intersection_grid[node_id][time_id].append((drone_fn, t))


    '''DETECTION OF INTERACTIONS
    Now it is simply necessary to loop over the grid and for every node_id and every time_id 
    then verify all trajectory pairs in this cell and neighbouring ones
    i.e. one time_id before and after are potential interactions
    * If we want to avoid to conservative consideration of intersections, we need to filter further 
    all pairs by the actual time separation whether it is less than delta_time_horizontal
    To avoid duplicates, for every time_id we check all combinations of:
    * trajs in time_id
    * trajs in time_id vs time_id+1
    
    Create interaction for any filtered pair of drones'''
    interactions = set()
    for node_id in horizontal_intersection_grid:
        for time_id in horizontal_intersection_grid[node_id]:

            potential_intersections = list(itertools.combinations(horizontal_intersection_grid[node_id][time_id], 2))
            if time_id + 1 in horizontal_intersection_grid[node_id]:
                potential_intersections.extend(itertools.product(horizontal_intersection_grid[node_id][time_id],
                                                                 horizontal_intersection_grid[node_id][time_id + 1]))

            for pair in potential_intersections:
                if pair[0][0] != pair[1][0]:  # not the same flight intention i.e. drone
                    if abs(pair[0][1] - pair[1][1]) <= delta_time_horizontal:  # further fillterig per actual separation
                        interactions.add((pair[0][0], pair[1][0]))

    return interactions


def generate_parallel_trajectories(drone, model, step, dist, number_to_generate, geofence_time_intervals):
    trajectories = []
    drone_dep_dual_list = []
    for node in drone.dep:
        drone_dep_dual = ("S" + node, node)
        drone_dep_dual_list.append(drone_dep_dual)
    drone_arr_dual_list = []
    for node in drone.arr:
        drone_arr_dual = (node, node + "T")
        drone_arr_dual_list.append(drone_arr_dual)
    if len(drone_dep_dual_list) < 2:
        print("TOOSHORT1")
        print(drone_dep_dual_list)
    # print(drone_dep_dual, drone_arr_dual)
    shortest_path = a2.astar_dual(model, drone_dep_dual_list, drone_arr_dual_list, drone, drone.dep_time)

    shortest_path.new_set_path(shortest_path.path, model)

    # shortest_path.set_path(shortest_path.path, model)

    #Check that there are no nodes in the geofenced_nodes list
    # protection_area, nb_FL, delay_max, FL_sep, FL_min, temps_sep_vertiport = model.generation_params
    max_time = drone.dep_time + max(shortest_path.path_dict.keys()) + model.delay_max + model.nb_FL * model.FL_sep / Drone.vertical_speed
    modified_edges_primal, modified_edges_dual = dict(), dict()
    at_least_one_edge_modified = False
    for interval in geofence_time_intervals:
        # Then there's potentially a geofence breach
        if drone.dep_time <= interval[0] <= max_time or drone.dep_time <= interval[1] <= max_time:
            for node in geofence_time_intervals[interval]:
                # Edges to modify in primal graph
                edges_primal = []
                for edge in model.graph.edges(node):
                    # Make sure that the edge is always written the same way since we are in an undirected graph
                    if edge[0] > edge[1]:
                        edge = (edge[1], edge[0])
                    if edge not in modified_edges_primal:
                        edges_primal.append(edge)
                # Edges to modify in dual graph
                edges_dual = []
                for edge in edges_primal:
                    for edge_dual_in in model.graph_dual.in_edges(edge):
                        if edge_dual_in not in modified_edges_dual:
                            edges_dual.append(edge_dual_in)
                    for edge_dual_out in model.graph_dual.out_edges(edge):
                        if edge_dual_out not in modified_edges_dual:
                            edges_dual.append(edge_dual_out)

                # Modify and save original length
                for edge in edges_primal:
                    if edge in modified_edges_primal:
                        pass
                    else:
                        modified_edges_primal[edge] = model.graph.edges[edge]["length"]
                        model.graph.edges[edge]["length"] = model.graph.edges[edge]["length"] * 100
                for edge_dual in edges_dual:
                    if edge_dual in modified_edges_dual:
                        pass
                    else:
                        modified_edges_dual[edge_dual] = model.graph_dual.edges[edge_dual]["length"]
                        model.graph_dual.edges[edge_dual]["length"] = model.graph_dual.edges[edge_dual]["length"] * 100

    if len(modified_edges_dual.keys()) > 0 or len(modified_edges_primal.keys()) > 0:
        at_least_one_edge_modified = True
    # If at least one edge was modified then we need to recompute the shortest path
    if at_least_one_edge_modified:
        shortest_path = a2.astar_dual(model, drone_dep_dual_list, drone_arr_dual_list, drone, drone.dep_time)

    remove_first = False
    for _i in range(1, len(shortest_path.path) - 1):
        if [shortest_path.path[_i - 1], shortest_path.path[_i]] == [drone.dep_edge[0], drone.dep_edge[1]]:
            remove_first = True
        if [shortest_path.path[_i - 1], shortest_path.path[_i]] == [drone.dep_edge[1], drone.dep_edge[0]]:
            remove_first = True
    if remove_first:
        if len(set(shortest_path.path[1:])) != len(shortest_path.path[1:]):
            print("ATTENTION1")
            if at_least_one_edge_modified:
                print("AND ONE EDGE MODIFIED")
            else:
                print("NO EDGES MODIFIED")
        trajectories.append(shortest_path.path[1:])
    else:
        if len(set(shortest_path.path)) != len(shortest_path.path):
            print("ATTENTION2")
            if at_least_one_edge_modified:
                print("AND ONE EDGE MODIFIED")
            else:
                print("NO EDGES MODIFIED")
        trajectories.append(shortest_path.path)
    # GENERATE ALL THE OTHER TRAJS
    geodesic = pyproj.Geod(ellps='WGS84')
    x_dep, y_dep = drone.departure_vertiport
    x_arr, y_arr = drone.arrival_vertiport

    heading, _back_azimuth1, _distance = geodesic.inv(x_dep, y_dep, x_arr, y_arr)
    nodes_to_deviate_from = []
    for i in range(1, step + 1):
        nodes_to_deviate_from.append(shortest_path.path[i * (len(shortest_path.path) // (step + 1))])
    count = 0
    tries = 0
    while len(trajectories) < number_to_generate and tries < 20:
        tries += 1
        nodes_to_visit = []

        # Define the heading in which to deviate
        if count % 2 == 1:
            new_heading = (heading + 90) % 360
        else:
            new_heading = (heading - 90) % 360

        for node in nodes_to_deviate_from:
            # generer un noeud dans une direction
            hash_nodes, hash_edges, min_x, min_y, x_step, y_step, resolution = model.hash_map
            pt = (model.graph.nodes[node]["x"], model.graph.nodes[node]["y"])
            north_factor = math.cos(new_heading)
            east_factor = math.sin(new_heading)
            new_pt = tools.m_displacement_to_lat_lon(pt, north_factor * dist * (1 + count // 2),
                                                     east_factor * dist * (1 + count // 2))
            new_x = new_pt[0]
            new_y = new_pt[1]
            new_node = tools.find_closest_node_with_hash(new_x, new_y, hash_nodes, min_x, min_y, x_step, y_step,
                                                         resolution, model.graph)
            if new_node in drone.arr:
                continue
            if new_node in drone.dep:
                continue
            nodes_to_visit.append(new_node)
        # Check that at least 1 node to visit was generated
        if len(nodes_to_visit) > 0:
            # get closest node
            drone_dep_dual_list = []
            for node in drone.dep:
                drone_dep_dual = ("S" + node, node)
                drone_dep_dual_list.append(drone_dep_dual)
            drone_arr_dual_list = [(nodes_to_visit[0], nodes_to_visit[0] + "T")]
            trajectory = a2.astar_dual(model, drone_dep_dual_list, drone_arr_dual_list, drone, drone.dep_time).path
            for i in range(len(nodes_to_visit) - 1):
                node = nodes_to_visit[i]
                next_node = nodes_to_visit[i + 1]
                drone_dep_dual_list = [("S" + node, node)]
                drone_arr_dual_list = [(next_node, next_node + "T")]
                _path = a2.astar_dual(model, drone_dep_dual_list, drone_arr_dual_list, drone, drone.dep_time).path[1:]
                trajectory += _path
            drone_dep_dual_list = [("S" + nodes_to_visit[-1], nodes_to_visit[-1])]
            drone_arr_dual_list = []
            for node in drone.arr:
                drone_arr_dual = (node, node + "T")
                drone_arr_dual_list.append(drone_arr_dual)
            trajectory += a2.astar_dual(model, drone_dep_dual_list, drone_arr_dual_list, drone, drone.dep_time).path[1:]
            trajectory_ok = True

            if len(set(trajectory)) != len(trajectory):
                index = 1
                while index < len(trajectory) - 2:
                    if trajectory[index] == trajectory[index + 1]:
                        trajectory.pop(index)
                    if trajectory[index - 1] == trajectory[index + 1]:
                        trajectory.pop(index)
                        trajectory.pop(index)
                    index += 1
                # print(trajectory)
                if len(set(trajectory)) != len(trajectory):
                    trajectory_ok = False
                if not check_traj_is_possible(model.graph, trajectory):
                    print("TRAJ impossible")
                    trajectory_ok = False

            # TODO faire plus rapide ou juste faire que ça arrive pas
            # Check dep_edge isn't in traj
            for _i in range(1, len(trajectory)-1):
                if [trajectory[_i-1], trajectory[_i]] == [drone.dep_edge[0], drone.dep_edge[1]]:
                    trajectory_ok = False
                if [trajectory[_i-1], trajectory[_i]] == [drone.dep_edge[1], drone.dep_edge[0]]:
                    trajectory_ok = False

            if (trajectory not in trajectories) and trajectory_ok:
                trajectories.append(trajectory)
        count += 1

    # Set the edges back to their initial values :
    if at_least_one_edge_modified:
        for edge in modified_edges_primal:
            # print("BEFORE :", model.graph.edges[edge]["length"])
            model.graph.edges[edge]["length"] = modified_edges_primal[edge]
            # print("AFTER :", model.graph.edges[edge]["length"])
        for edge_dual in modified_edges_dual:
            model.graph_dual.edges[edge_dual]["length"] = modified_edges_dual[edge_dual]

    return trajectories


def return_drone_from_flight_number(model, flight_number):
    return model.total_drone_dict[flight_number]


def check_traj_is_possible(graph, traj):
    for index in range(len(traj) - 1):
        if (traj[index], traj[index + 1]) not in graph.edges and (traj[index + 1], traj[index]) not in graph.edges:
            return False
    return True
'''
def generate_one_point_trajectories(drone, graph, graph_dual, points_to_explore, model):
    trajectories = []
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
            trajectory.pop(len(first_part) - 1)
            trajectory.pop(len(first_part) - 1)
            if not check_traj_is_possible(graph, trajectory):
                if trajectory not in trajectories:
                    trajectories.append(trajectory)
    return trajectories


def generate_multiple_point_trajectories(drone, graph, list_of_points_to_explore, model, nb_traj_to_keep):
    trajectories = []
    drone_dep_dual = ("S" + drone.dep, drone.dep)
    drone_arr_dual = (drone.arr, drone.arr + "T")
    # print(drone_dep_dual, drone_arr_dual)
    shortest_path = a2.astar_dual(model, drone_dep_dual, drone_arr_dual, drone, drone.dep_time)
    # print(shortest_path.path)
    trajectories.append(shortest_path.path)
    waypoints_list = [[drone.dep]]
    already_calculated_intervals_dict = dict()
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
            arr_dual = (waypoints[waypoint_index + 1], waypoints[waypoint_index + 1] + "T")
            if (dep_dual, arr_dual) in already_calculated_intervals_dict:
                path = already_calculated_intervals_dict[(dep_dual, arr_dual)]
            elif (arr_dual, dep_dual) in already_calculated_intervals_dict:
                path = already_calculated_intervals_dict[(arr_dual, dep_dual)]
            else:
                path = a2.astar_dual(model, dep_dual, arr_dual, drone, drone.dep_time).path
                already_calculated_intervals_dict[(dep_dual, arr_dual)] = path
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
    # Remove duplicates
    for index in range(len(trajectories) - 1, 0, -1):
        if trajectories[index] in trajectories[0:index]:
            trajectories.pop(index)
    # Keep only the right number of trajectories
    while len(trajectories) > nb_traj_to_keep:
        trajectories.pop(-1)
    # print("len :", len(trajectories))
    return trajectories


def generate_points_from_shortest_path(model, drone, graph, steps, depth):
    # Create 2 nodes combinaisons from the shortest_path nodes' succesors
    trajectories = []
    # 1 A* to find the shortest path
    drone_dep_dual = ("S" + drone.dep, drone.dep)
    drone_arr_dual = (drone.arr, drone.arr + "T")
    shortest_path = a2.astar_dual(model, drone_dep_dual, drone_arr_dual, drone, drone.dep_time).path
    index_list = [(i + 1) * (len(shortest_path) // (steps + 1)) for i in range(steps)]
    nodes_list = []
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


def generate_points_to_pass_by_for_one_point(graph, steps):
    # Quadriller l'espace
    # Find max x,y et min x,y
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
    x_range = [min_x + i * (max_x - min_x) / (steps - 1) for i in range(steps)]
    y_range = [min_y + i * (max_y - min_y) / (steps - 1) for i in range(steps)]
    x_y_pos = [[(x, y) for y in y_range] for x in x_range]
    for x in range(len(x_y_pos)):
        for y in range(len(x_y_pos[0])):
            best_dist = 5000000
            closest_node = None
            for node in graph.nodes:
                dist = math.sqrt(
                    (x_y_pos[x][y][0] - graph.nodes[node]["x"]) ** 2 + (x_y_pos[x][y][1] - graph.nodes[node]["y"]) ** 2)
                if dist < best_dist:
                    closest_node = node
                    best_dist = dist
            # nodes_to_explore[x][y] = closest_node
            nodes_to_explore.append(closest_node)
    return set(nodes_to_explore)
'''
