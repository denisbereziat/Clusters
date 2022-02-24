"""Generate trajectories to be used in the optim model"""

# TODO Encore quelques problemes avec certains drones particuliers ou on arriv pas a generer de traj alternatives

import math
import tools
import Astar2 as a2
import Path
import pyproj
import matplotlib.pyplot as plt
import itertools
import Drone
import sys


# # graph_file_path = "graph_files/processed_graphM2.graphml"
# graph_file_path = "graph_files/total_graph_200m.graphml"
# dual_graph_path = "graph_files/dual_total_graph_200m.graphml"
# # graph_file_path = "graph_files/geo_data/crs_epsg_32633/road_network/crs_4326_cleaned_simplified_network/cleaned_simplified.graphml"
# # drone_list_file_path = 'graph_files/drones.txt'
# # drone_list_file_path = 'graph_files/test_flight_intention.csv'
# drone_list_file_path = 'graph_files/Intentions/100_flight_intention.csv'
# # vertical_protection_area = 7.62 # 25 ft


def generate_trajectories(model, graph, raw_graph, graph_dual, current_param=None):
    """Generate alternative trajectories for all drones in the model
    OUTPUT :
    drone_trajectories_dict[drone_fn][traj_id] = [path_object, total_time]
    trajectories_to_path[traj_id] = path
    trajectories_to_fn[traj_id] = drone_fn
    trajectories_to_duration[traj_id] = total_time
    fn_order = list of flight_number of the list of model.droneList
    """

    protection_area, nb_FL, delay_max, FL_sep, FL_min, temps_sep_vertiport = model.generation_params
    if current_param is not None:
        drone_trajectories_dict, trajectories_to_fn, trajectories_to_duration, trajectories_to_path, fn_order = current_param
    else:
        drone_trajectories_dict, trajectories_to_fn, trajectories_to_duration, trajectories_to_path, fn_order = dict(), dict(), dict(), dict(), []
    nb_alternative_traj = 5

    generated_trajectories = dict()
    # Generate alternate trajectories for all drones.
    # print("Trajectories generation")
    # If the drone doesn't already have trajectories generated we generate some
    for drone in model.droneList:
        if drone.flight_number not in drone_trajectories_dict:
            generated_trajectories[drone.flight_number] = generate_parallel_trajectories(drone, graph, model, 6, 500,
                                                                                         nb_alternative_traj)
        else:
            pass
        # print("drone already has generated traj")

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
            path.set_path(traj, model)
            total_time = max(list(path.path_dict.keys()))
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


def save_intersection(trajectories_to_path, k1, k2, multiPoints, multiPointsStatus, horizontal_shared_nodes_list):
    # check what is situation and save intersection(s)
    # print("Connected :", multiPoints)
    if multiPointsStatus == 1:  # this could be verified with len of multiPoints
        # save independent intersecting point
        t1 = multiPoints[0][1]
        t2 = multiPoints[0][2]
        conflict = (k1, k2, t1, t2, max(trajectories_to_path[k1].separation_dict[t1][1],
                                        trajectories_to_path[k2].separation_dict[t2][0]),
                    max(trajectories_to_path[k2].separation_dict[t2][1],
                        trajectories_to_path[k1].separation_dict[t1][0]))
    else:
        first_node = multiPoints[0]
        last_node = multiPoints[-1]

        t_k1_first = first_node[1]
        t_k1_last = last_node[1]

        t_k2_first = first_node[2]
        t_k2_last = last_node[2]

        sep12_first = max(trajectories_to_path[k1].separation_dict[t_k1_first][1],
                          trajectories_to_path[k2].separation_dict[t_k2_first][0])
        sep12_last = max(trajectories_to_path[k1].separation_dict[t_k1_last][1],
                         trajectories_to_path[k2].separation_dict[t_k2_last][0])

        sep21_first = max(trajectories_to_path[k2].separation_dict[t_k2_first][1],
                          trajectories_to_path[k1].separation_dict[t_k1_first][0])
        sep21_last = max(trajectories_to_path[k2].separation_dict[t_k2_last][1],
                         trajectories_to_path[k1].separation_dict[t_k1_last][0])

        if multiPointsStatus == 2:
            # they are consecutive, take fist and last and create single intersection condition
            cut_k2 = (t_k1_last - t_k1_first) - (t_k2_last - t_k2_first)
            delta_sep12 = sep12_last - sep12_first

            new_sep12_first = sep12_first + max(0, min(cut_k2, cut_k2 + delta_sep12)) + max(0, min(delta_sep12,
                                                                                                   delta_sep12 + cut_k2))

            cut_k1 = (t_k2_last - t_k2_first) - (t_k1_last - t_k1_first)
            delta_sep21 = sep21_last - sep21_first

            new_sep21_first = sep21_first + max(0, min(cut_k1, cut_k1 + delta_sep21)) + max(0, min(delta_sep21,
                                                                                                   delta_sep21 + cut_k1))
        elif multiPointsStatus == 3:
            # they are opposite
            new_sep12_first = (t_k1_last - t_k1_first) + (t_k2_first - t_k2_last) + sep12_last
            new_sep21_first = sep21_first
        conflict = (k1, k2, t_k1_first, t_k2_first, new_sep12_first, new_sep21_first)

    horizontal_shared_nodes_list.append(conflict)
    # print("conflict MultipointStatus", multiPointsStatus, " : ", conflict)


def generate_intersection_points(drone_trajectories_dict, trajectories_to_fn_dict, trajectories_to_path, model, graph,
                                 graph_dual, raw_graph):
    """Generate intersection points between given trajectories"""

    protection_area, nb_FL, delay_max, FL_sep, FL_min, temps_sep_vertiport = model.generation_params
    max_delta_fl = nb_FL - 1
    horizontal_shared_nodes_list = []
    climb_horiz_list = []
    descent_horiz_list = []
    climb_climb_list = []
    descent_descent_list = []

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
    delta_time_horizontal = math.ceil(delay_max + model.protection_area / Drone.speeds_dict_model1["turn2"])
    horizontal_intersection_grid = {}
    # vertical-horizontal
    delta_time_clmb_hor = math.ceil(
        delay_max + Drone.vertical_speed / (2 * Drone.vertical_accel) + model.protection_area / Drone.speeds_dict_model1[
            "turn2"])
    clmb_intersection_grid = {}
    clmb_hor_intersection_grid = {}
    delta_time_desc_hor = math.ceil(
        delay_max + 2 * max_delta_fl * FL_sep / Drone.vertical_speed + Drone.vertical_speed / (
                2 * Drone.vertical_accel) + model.protection_area / Drone.speeds_dict_model1["turn2"])
    desc_intersection_grid = {}
    desc_hor_intersection_grid = {}
    # vertical-vertical
    delta_time_dep = math.ceil(delay_max + temps_sep_vertiport)
    dep_intersection_grid = {}
    delta_time_arr = math.ceil(delay_max + 2 * max_delta_fl * FL_sep / Drone.vertical_speed + temps_sep_vertiport)
    arr_intersection_grid = {}
    '''Loop over all drones, trajectories and their nodes and fill the grids 
    by adding concerned trajectories and respective passage time over intersecting point (and any other required data)
    * horizontal_intersection_grid indexed by nodes and normalized time
    * clmb_hor_intersection_grid and desc_hor_intersection_grid indexed by edge_id and normalized time
    (edge_id is the one in which middle the vertiport is located)
    * dep_intersection_grid and arr_intersection_grid indexed by vertiport_id and normalized time'''
    for drone in model.droneList:
        # initialize clmb_intersection_grid
        dep_edge = drone.dep_edge
        if dep_edge:  # vertiport in the unconstrained airspace are not linked with edge, hence no intersection
            if dep_edge[0] > dep_edge[1]:
                dep_edge = (dep_edge[1], dep_edge[0])
            if not dep_edge in clmb_intersection_grid:
                clmb_intersection_grid[dep_edge] = {}
        # initialize desc_intersection_grid
        arr_edge = drone.arr_edge
        if arr_edge:  # vertiport in the unconstrained airspace are not linked with edge, hence no intersection
            if arr_edge[0] > arr_edge[1]:
                arr_edge = (arr_edge[1], arr_edge[0])
            if not arr_edge in desc_intersection_grid:
                desc_intersection_grid[arr_edge] = {}
        # initialize dep_intersection_grid
        dep = drone.dep
        # TODO dep vertiport plutot
        if not dep in dep_intersection_grid:
            dep_intersection_grid[dep] = {}
        # initialize arr_intersection_grid
        arr = drone.arr
        if not arr in arr_intersection_grid:
            arr_intersection_grid[arr] = {}

        drone_fn = drone.flight_number
        for traj_id in drone_trajectories_dict[drone_fn]:
            traj_path = drone_trajectories_dict[drone_fn][traj_id][0]
            # fill clmb_intersection_grid
            if dep_edge:
                time_id = traj_path.dep_time // delta_time_clmb_hor
                if not time_id in clmb_intersection_grid[dep_edge]:
                    clmb_intersection_grid[dep_edge][time_id] = []
                clmb_intersection_grid[dep_edge][time_id].append((traj_id, traj_path.dep_time))
            # fill desc_intersection_grid
            if arr_edge:
                time_id = traj_path.arr_time // delta_time_desc_hor
                if not time_id in desc_intersection_grid[arr_edge]:
                    desc_intersection_grid[arr_edge][time_id] = []
                desc_intersection_grid[arr_edge][time_id].append((traj_id, traj_path.arr_time))
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

            for t, node_id in traj_path.path_dict.items():
                # initialize horizontal_intersection_grid
                time_id = t // delta_time_horizontal  # integer division
                if not node_id in horizontal_intersection_grid:
                    horizontal_intersection_grid[node_id] = {}
                if not time_id in horizontal_intersection_grid[node_id]:
                    horizontal_intersection_grid[node_id][time_id] = []
                # fill horizontal_intersection_grid
                horizontal_intersection_grid[node_id][time_id].append((traj_id, t))

            # !!!ATTENTION!!!We could avoid this second loop if path_dict is already ordered by time and then edges are created on the fly
            for edge_id, (t, speed) in traj_path.edge_middle_dict.items():
                if edge_id[0] > edge_id[1]:
                    edge_id = (edge_id[1], edge_id[0])
                # initialize clmb_hor_intersection_grid
                time_id = t // delta_time_clmb_hor
                if not edge_id in clmb_hor_intersection_grid:
                    clmb_hor_intersection_grid[edge_id] = {}
                if not time_id in clmb_hor_intersection_grid[edge_id]:
                    clmb_hor_intersection_grid[edge_id][time_id] = []
                # fill clmb_hor_intersection_grid
                clmb_hor_intersection_grid[edge_id][time_id].append((traj_id, t, speed))

                # initialize desc_hor_intersection_grid
                time_id = t // delta_time_desc_hor
                if not edge_id in desc_hor_intersection_grid:
                    desc_hor_intersection_grid[edge_id] = {}
                if not time_id in desc_hor_intersection_grid[edge_id]:
                    desc_hor_intersection_grid[edge_id][time_id] = []
                # fill desc_hor_intersection_grid
                desc_hor_intersection_grid[edge_id][time_id].append((traj_id, t, speed))

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
        # print("k1 k2")
        # print(k1, k2)

        if len(points) == 1:
            # this is just simple intersection
            # save intersecting point and continue
            # print("points initial")
            # print(points)
            save_intersection(trajectories_to_path, k1, k2, points, 1, horizontal_shared_nodes_list)
        # t1 = points[0][1]
        # t2 = points[0][2]
        # conflict = (k1, k2, t1, t2, protection_area / trajectories_to_path[k1].speed_time_stamps[t1], protection_area / trajectories_to_path[k2].speed_time_stamps[t2])
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
                        if (multiPoints[-1][2], multiPoints[-1][0]) in trajectories_to_path[k2].next_node_dict:
                            if trajectories_to_path[k2].next_node_dict[(multiPoints[-1][2], multiPoints[-1][0])] == (
                                    points[point_index][2], points[point_index][0]):
                                # consecutive multiple points
                                multiPoints.append(points[point_index])
                                multiPointsStatus = 2
                                point_index += 1
                                continue
                        if (multiPoints[-1][2], multiPoints[-1][0]) in trajectories_to_path[k2].previous_node_dict:
                            if trajectories_to_path[k2].previous_node_dict[
                                (multiPoints[-1][2], multiPoints[-1][0])] == (
                                    points[point_index][2], points[point_index][0]):
                                # opposite multiple points
                                multiPoints.append(points[point_index])
                                multiPointsStatus = 3
                                point_index += 1
                                continue

                # multi point chain is broken
                # save intersection(s)
                save_intersection(trajectories_to_path, k1, k2, multiPoints, multiPointsStatus,
                                  horizontal_shared_nodes_list)
                # print("Connected :", multiPoints)
                # if multiPointsStatus == 1: #this could be verified with len of multiPoints
                ##save independent intersecting point
                # t1 = multiPoints[0][1]
                # t2 = multiPoints[0][2]
                # conflict = (k1, k2, t1, t2, protection_area / trajectories_to_path[k1].speed_time_stamps[t1], protection_area / trajectories_to_path[k2].speed_time_stamps[t2])
                # else:
                # first_node = multiPoints[0]
                # last_node = multiPoints[-1]

                # t_k1_first = first_node[1]
                # t_k1_last = last_node[1]
                # sep12_first = protection_area / trajectories_to_path[k1].speed_time_stamps[t_k1_first]
                # sep12_last = protection_area / trajectories_to_path[k1].speed_time_stamps[t_k1_last]

                # t_k2_first = first_node[2]
                # t_k2_last = last_node[2]
                # sep21_first = protection_area / trajectories_to_path[k2].speed_time_stamps[t_k2_first]
                # sep21_last = protection_area / trajectories_to_path[k2].speed_time_stamps[t_k2_last]

                # if multiPointsStatus == 2:
                ##they are consecutive, take fist and last and create single intersection condition
                # cut_k2 = (t_k1_last - t_k1_first) - (t_k2_last - t_k2_first)
                # delta_sep12 = sep12_last - sep12_first

                # new_sep12_first = sep12_first + max(0, min(cut_k2, cut_k2 + delta_sep12)) + max(0, min(delta_sep12, delta_sep12 + cut_k2))

                # cut_k1 = (t_k2_last - t_k2_first) - (t_k1_last - t_k1_first)
                # delta_sep21 = sep21_last - sep21_first

                # new_sep21_first = sep21_first + max(0, min(cut_k1, cut_k1 + delta_sep21)) + max(0, min(delta_sep21, delta_sep21 + cut_k1))
                # elif multiPointsStatus == 3:
                ##they are opposite
                # new_sep12_first = (t_k1_last - t_k1_first) + (t_k2_first - t_k2_last) + sep12_last
                # new_sep21_first = sep21_first
                # conflict = (k1, k2, t_k1_first, t_k2_first, new_sep12_first, new_sep21_first)

                # horizontal_shared_nodes_list.append(conflict)
                # print("conflict MultipointStatus", multiPointsStatus, " : ", conflict)

                # reinitialise multiPoints and multiPointsStatus
                multiPoints = [points[point_index]]
                multiPointsStatus = 1
                point_index += 1

            # make last saving
            save_intersection(trajectories_to_path, k1, k2, multiPoints, multiPointsStatus,
                              horizontal_shared_nodes_list)

    '''DETECTION OF VERT_HOR INTERSECTION POINTS
	Loop over the vertical greed (containing all clmb and desc trajs) and for every edge_id and every time_id 
	verify if vertical trajs are in interaction with horizontal in the neighbourhood.
	To avoid duplicates for every time_id we check all combinations of:
	* vert[time_id] vs hor[time_id]
	* vert[time_id] vs hor[time_id+1]
	* vert[time_id+1] vs hor[time_id]
	The principle is the same for clmb_hor_intersection_grid and desc_hor_intersection_grid just time norm is different'''

    for dep_edge in clmb_intersection_grid:
        for time_id in clmb_intersection_grid[dep_edge]:
            if dep_edge in clmb_hor_intersection_grid:  # if no hor drones no intersections
                potential_intersections = []
                if time_id in clmb_hor_intersection_grid[dep_edge]:
                    # add interaction of current clmb and hor
                    potential_intersections.extend(itertools.product(clmb_intersection_grid[dep_edge][time_id],
                                                                     clmb_hor_intersection_grid[dep_edge][time_id]))
                    # add interaction of next clmb and current hor
                    if time_id + 1 in clmb_intersection_grid[dep_edge]:
                        potential_intersections.extend(itertools.product(clmb_intersection_grid[dep_edge][time_id + 1],
                                                                         clmb_hor_intersection_grid[dep_edge][time_id]))

                # add interaction of current clmb and next hor
                if time_id + 1 in clmb_hor_intersection_grid[dep_edge]:
                    potential_intersections.extend(itertools.product(clmb_intersection_grid[dep_edge][time_id],
                                                                     clmb_hor_intersection_grid[dep_edge][time_id + 1]))

                for pair in potential_intersections:
                    if trajectories_to_fn_dict[pair[0][0]] != trajectories_to_fn_dict[pair[1][0]]:
                        if abs(pair[0][1] - pair[1][1]) <= delta_time_clmb_hor:  # further fillterig per actual separation
                            # create intersecting point
                            conflict = (pair[0][0], pair[1][0], pair[0][1], pair[1][1], temps_sep_vertiport,
                                        protection_area / pair[1][2])
                            climb_horiz_list.append(conflict)

    '''DETECTION OF VERTICAL INTERSECTION POINTS
	Loop over the dep_intersection_grid and for every dep point and every time_id 
	verify all pairs of trajectories in the neighbourhood.
	To avoid duplicates for every time_id we check all combinations:
	* in deps[time_id]
	* of deps[time_id] vs deps[time_id+1]
	The principle is the same for arr_intersection_grid just time norm is different.'''

    # DETECTION OF DEP INTERSECTION POINTS
    for dep in dep_intersection_grid:
        for time_id in dep_intersection_grid[dep]:
            potential_intersections = list(itertools.combinations(dep_intersection_grid[dep][time_id], 2))
            if time_id + 1 in dep_intersection_grid[dep]:
                potential_intersections.extend(
                    itertools.product(dep_intersection_grid[dep][time_id], dep_intersection_grid[dep][time_id + 1]))

            for pair in potential_intersections:
                if trajectories_to_fn_dict[pair[0][0]] != trajectories_to_fn_dict[pair[1][0]]:
                    if abs(pair[0][1] - pair[1][1]) <= delta_time_dep:  # further fillterig per actual separation
                        # create intersecting point
                        conflict = (
                            pair[0][0], pair[1][0], pair[0][1], pair[1][1], temps_sep_vertiport, temps_sep_vertiport)
                        climb_climb_list.append(conflict)

    # DETECTION OF ARR INTERSECTION POINTS
    for arr in arr_intersection_grid:
        for time_id in arr_intersection_grid[arr]:
            potential_intersections = list(itertools.combinations(arr_intersection_grid[arr][time_id], 2))
            if time_id + 1 in arr_intersection_grid[arr]:
                potential_intersections.extend(
                    itertools.product(arr_intersection_grid[arr][time_id], arr_intersection_grid[arr][time_id + 1]))

            for pair in potential_intersections:
                if trajectories_to_fn_dict[pair[0][0]] != trajectories_to_fn_dict[pair[1][0]]:
                    if abs(pair[0][1] - pair[1][1]) <= delta_time_arr:  # further fillterig per actual separation
                        # create intersecting point
                        conflict = (
                            pair[0][0], pair[1][0], pair[0][1], pair[1][1], temps_sep_vertiport, temps_sep_vertiport)
                        descent_descent_list.append(conflict)
    return horizontal_shared_nodes_list, climb_horiz_list, descent_horiz_list, climb_climb_list, descent_descent_list
    # sys.exit(0)
    #
    # size = 0
    # for drone_fn in drone_trajectories_dict:
    #     for traj in drone_trajectories_dict[drone_fn]:
    #         size += 1
    # ####
    # # Check for shared nodes between trajectories
    # # print("Check for shared nodes on horizontal route")
    # # print("size", size)
    #
    # first_conflict = None
    # conflict_id = None
    # who_was_first = None
    # count_fused_nodes = 0
    # for i in range(size):
    #     # print(i)
    #     for j in range(i + 1, size):
    #         flight_number1 = trajectories_to_fn_dict[i]
    #         flight_number2 = trajectories_to_fn_dict[j]
    #         if flight_number1 == flight_number2:
    #             continue
    #         drone1 = return_drone_from_flight_number(model, flight_number1)
    #         drone2 = return_drone_from_flight_number(model, flight_number2)
    #         drone1.path_object = drone_trajectories_dict[flight_number1][i][0]
    #         drone2.path_object = drone_trajectories_dict[flight_number2][j][0]
    #         traj1 = i
    #         traj2 = j
    #
    #         # Nodes that both trajectories use
    #         consecutive_nodes = [drone1.path_object.path[0]]
    #         for index1, node in enumerate(drone1.path_object.path):
    #             if node in drone2.path_object.path:
    #                 index2 = drone2.path_object.path.index(node)
    #                 t1 = None
    #                 for t in drone1.path_object.path_dict:
    #                     if drone1.path_object.path_dict[t] == node:
    #                         t1 = t
    #                         break
    #                 t2 = None
    #                 for t in drone2.path_object.path_dict:
    #                     if drone2.path_object.path_dict[t] == node:
    #                         t2 = t
    #                         break
    #                 v1 = get_drone_speed_after_node(model, drone1, graph, graph_dual, node)
    #                 v2 = get_drone_speed_after_node(model, drone2, graph, graph_dual, node)
    #
    #                 # conflict = (traj1, traj2, t1, t2, protection_area / v1, protection_area / v2)
    #                 # if check_if_conflict_is_possible(conflict, model):
    #                 #     horizontal_shared_nodes_list.append(conflict)
    #
    #                 # drone_shared_nodes_tab[i][j].append((t1, t2, protection_area/v1, protection_area/v2))
    #                 t_stamps_d1 = sorted(list(drone1.path_object.path_dict.keys()))
    #                 t_stamps_d2 = sorted(list(drone2.path_object.path_dict.keys()))
    #                 # Same way
    #                 if t1 != t_stamps_d1[-1] and t2 != t_stamps_d2[-1]:
    #                     d1_t_next_node = t_stamps_d1[t_stamps_d1.index(t1) + 1]
    #                     d2_t_next_node = t_stamps_d2[t_stamps_d2.index(t2) + 1]
    #                     d1_next_node = drone1.path_object.path_dict[d1_t_next_node]
    #                     d2_next_node = drone2.path_object.path_dict[d2_t_next_node]
    #                     # same way
    #                     if d1_next_node == d2_next_node:
    #                         d1_previous_node = drone1.path_object.path[index1 - 1]
    #                         d2_previous_node = drone2.path_object.path[index2 - 1]
    #                         # if t1 <= t2:
    #                         # Check it's the same node, and same order and consecutive
    #                         if d1_previous_node == d2_previous_node and d1_previous_node in consecutive_nodes and who_was_first == drone1.flight_number:
    #                             # delta_2 = max(protection_area / v1, (t1 - d1_t_next_node) - (t2 - d2_t_next_node))
    #                             delta_1 = max(protection_area / v1,
    #                                           protection_area / v1 + (t2 - d2_t_next_node) - (t1 - d1_t_next_node))
    #                             delta_2 = max(protection_area / v2,
    #                                           protection_area / v2 + (t1 - d1_t_next_node) - (t2 - d2_t_next_node))
    #                             conflict = (traj1, traj2, t1, t2, delta_1, delta_2)
    #                             consecutive_nodes.append(node)
    #                             cut2 = (t1 - first_conflict[2]) - (t2 - first_conflict[3])
    #                             delta_sep12 = conflict[4] - first_conflict[4]
    #                             cut1 = - cut2
    #                             delta_sep21 = conflict[5] - first_conflict[5]
    #                             new_sep12 = first_conflict[4] + max(min(cut2, cut2 + delta_sep12), 0) + max(
    #                                 min(delta_sep12, delta_sep12 + cut2), 0)
    #                             new_sep21 = first_conflict[5] + max(min(cut1, cut1 + delta_sep21), 0) + max(
    #                                 min(delta_sep21, delta_sep21 + cut1), 0)
    #                             conflict = (traj1, traj2, first_conflict[2], first_conflict[3], new_sep12, new_sep21)
    #                             horizontal_shared_nodes_list[conflict_id] = conflict
    #                             count_fused_nodes += 1
    #
    #                         else:
    #                             # If drone1 goes first we need to make sure the delta2 is enough for drone2 not to catch up
    #                             consecutive_nodes = [node]
    #                             # (t1-d1_t_next_node)-(t2-d2_t_next_node) > 0 if 2 is faster
    #                             # TODO A VERIFIER
    #                             delta_1 = max(protection_area / v1,
    #                                           protection_area / v1 + (t2 - d2_t_next_node) - (t1 - d1_t_next_node))
    #                             delta_2 = max(protection_area / v2,
    #                                           protection_area / v2 + (t1 - d1_t_next_node) - (t2 - d2_t_next_node))
    #                             conflict = (traj1, traj2, t1, t2, delta_1, delta_2)
    #                             first_conflict = conflict
    #                             if check_if_conflict_is_possible(conflict, model):
    #                                 horizontal_shared_nodes_list.append(conflict)
    #                                 conflict_id = len(horizontal_shared_nodes_list) - 1
    #                         who_was_first = drone1.flight_number
    #                     else:
    #                         # TODO EN FAIT c'eST MAX (V1/SEP , V2/SEP) avec un des deux qui est v apres le noeud et l'autre v avant le noeud
    #                         conflict = (traj1, traj2, t1, t2, protection_area / v1, protection_area / v2)
    #                         if check_if_conflict_is_possible(conflict, model):
    #                             horizontal_shared_nodes_list.append(conflict)
    #                             conflict_id = len(horizontal_shared_nodes_list) - 1
    #                 else:
    #                     conflict = (traj1, traj2, t1, t2, protection_area / v1, protection_area / v2)
    #                     if check_if_conflict_is_possible(conflict, model):
    #                         horizontal_shared_nodes_list.append(conflict)
    #                         conflict_id = len(horizontal_shared_nodes_list) - 1
    #
    #                     # if t2 < t1:
    #                     #     if d1_previous_node == d2_previous_node and d1_previous_node in consecutive_nodes and who_was_first == drone1.flight_number:
    #                     #         delta_1 = max(protection_area / v2, (t2 - d2_t_next_node) - (t1 - d1_t_next_node))
    #                     #         conflict = (traj1, traj2, t1, t2, delta_1, protection_area / v2)
    #                     #         consecutive_nodes.append(node)
    #                     #         cut2 = (t1 - first_conflict[2]) - (t2 - first_conflict[3])
    #                     #         delta_sep12 = conflict[4] - first_conflict[4]
    #                     #         cut1 = - cut2
    #                     #         delta_sep21 = conflict[5] - first_conflict[5]
    #                     #         new_sep12 = first_conflict[4] + max(min(cut2, cut2 + delta_sep12), 0) + max(min(delta_sep12, delta_sep12 + cut2), 0)
    #                     #         new_sep21 = first_conflict[5] + max(min(cut1, cut1 + delta_sep21), 0) + max(min(delta_sep21, delta_sep21 + cut1), 0)
    #                     #         conflict = (traj1, traj2, first_conflict[2], first_conflict[3], new_sep12, new_sep21)
    #                     #         horizontal_shared_nodes_list[conflict_id] = conflict
    #                     #         count_fused_nodes += 1
    #                     #     else:
    #                     #         # If drone2 goes first we need to make sure the delta1 is enough for drone2 not to catch up
    #                     #         consecutive_nodes = [node]
    #                     #         delta_1 = max(protection_area / v2, (t2 - d2_t_next_node) - (t1 - d1_t_next_node))
    #                     #         conflict = (traj1, traj2, t1, t2, delta_1, protection_area / v2)
    #                     #         if check_if_conflict_is_possible(conflict, model):
    #                     #             horizontal_shared_nodes_list.append(conflict)
    #                     #             conflict_id = len(horizontal_shared_nodes_list) - 1
    #                     #     who_was_first = drone2.flight_number
    #                 # else:
    #                 #     conflict = (traj1, traj2, t1, t2, protection_area / v1, protection_area / v2)
    #                 #     if check_if_conflict_is_possible(conflict, model):
    #                 #         horizontal_shared_nodes_list.append(conflict)
    #                 # else:
    #                 #     conflict = (traj1, traj2, t1, t2, protection_area / v1, protection_area / v2)
    #                 #     if check_if_conflict_is_possible(conflict, model):
    #                 #         horizontal_shared_nodes_list.append(conflict)
    #
    #                 # Opposite ways
    #                 # If D1 goes first
    #                 if t1 != t_stamps_d1[-1] and t2 != t_stamps_d2[0]:
    #                     if t1 < t2:
    #                         d1_t_next_node = t_stamps_d1[t_stamps_d1.index(t1) + 1]
    #                         d2_t_prev_node = t_stamps_d2[t_stamps_d2.index(t2) - 1]
    #                         d1_next_node = drone1.path_object.path_dict[d1_t_next_node]
    #                         d2_prev_node = drone2.path_object.path_dict[d2_t_prev_node]
    #                         if d1_next_node == d2_prev_node:
    #                             delta_1 = (t2 - d2_t_prev_node) + (d1_t_next_node - t1)
    #                             conflict = (traj1, traj2, t1, t2, delta_1, protection_area / v2)
    #                             if check_if_conflict_is_possible(conflict, model):
    #                                 horizontal_shared_nodes_list.append(conflict)
    #                 # if D2 goes first
    #                 if t2 != t_stamps_d2[-1] and t1 != t_stamps_d1[0]:
    #                     if t2 < t1:
    #                         d2_t_next_node = t_stamps_d2[t_stamps_d2.index(t2) + 1]
    #                         d1_t_prev_node = t_stamps_d1[t_stamps_d1.index(t1) - 1]
    #                         d2_next_node = drone2.path_object.path_dict[d2_t_next_node]
    #                         d1_prev_node = drone1.path_object.path_dict[d1_t_prev_node]
    #                         if d2_next_node == d1_prev_node:
    #                             delta_2 = (t1 - d1_t_prev_node) + (d2_t_next_node - t2)
    #                             conflict = (traj1, traj2, t1, t2, protection_area / v1, delta_2)
    #                             if check_if_conflict_is_possible(conflict, model):
    #                                 horizontal_shared_nodes_list.append(conflict)
    # # print("NB OF FUSED NODES = ", count_fused_nodes)
    # ####
    # # Horizontal/Vertical shared nodes
    # # TODO est ce qu'on veut les deux i,j j,i ou que un des deux suffit
    # # print("Searching for departure and arrival edges for each drone")
    # dep_edge_dict, arrival_edge_dict = get_all_dep_and_arr_edges(model, raw_graph)
    # # print("Check for vertically shared nodes with horizontal route")
    # for i in range(size):
    #     # print("D" + str(1 + (i // number_of_traj_to_keep)))
    #     flight_number1 = trajectories_to_fn_dict[i]
    #     drone1 = return_drone_from_flight_number(model, flight_number1)
    #     drone1.path_object = drone_trajectories_dict[flight_number1][i][0]
    #     dep_edge = dep_edge_dict[drone1.flight_number]
    #     arr_edge = arrival_edge_dict[drone1.flight_number]
    #     for j in range(size):
    #         flight_number2 = trajectories_to_fn_dict[j]
    #         if flight_number1 == flight_number2:
    #             continue
    #         drone2 = return_drone_from_flight_number(model, flight_number2)
    #         drone2.path_object = drone_trajectories_dict[flight_number2][j][0]
    #         # Find the edges the drone start and end on
    #         # Check if either is part of the other drone path
    #         # print(dep_edge)
    #         # TODO Faire les delta t
    #         for n in range(1, len(drone2.path_object.path)):
    #             if dep_edge == (drone2.path_object.path[n - 1], drone2.path_object.path[n]) or (
    #                     dep_edge[1], dep_edge[0]) == (drone2.path_object.path[n - 1], drone2.path_object.path[n]):
    #                 # k1, k2, t1, t2, delta1, delta2
    #                 # k1, k2 = i + 1, j + 1
    #                 k1, k2 = i, j
    #                 t1 = drone1.dep_time
    #                 drone2_t_stamps = sorted(list(drone2.path_object.path_dict.keys()))
    #                 # Use the expected time at half edge
    #                 t2 = (drone2_t_stamps[n - 1] + drone2_t_stamps[n]) / 2
    #                 delta1 = model.vertical_protection / drone1.vertical_speed
    #                 delta2 = model.protection_area / get_drone_speed_after_node(model, drone2, graph, graph_dual,
    #                                                                             drone2.path_object.path[n - 1])
    #                 conflict = (k1, k2, t1, t2, delta1, delta2)
    #                 if check_if_conflict_is_possible(conflict, model):
    #                     climb_horiz_list.append(conflict)
    #             if arr_edge == (drone2.path_object.path[n - 1], drone2.path_object.path[n]) or (
    #                     arr_edge[1], arr_edge[0]) == (drone2.path_object.path[n - 1], drone2.path_object.path[n]):
    #                 # k1, k2, t1, t2, delta1, delta2
    #                 k1, k2 = i, j
    #                 drone1_t_stamps = sorted(list(drone1.path_object.path_dict.keys()))
    #                 t1 = drone1_t_stamps[-1]
    #                 drone2_t_stamps = sorted(list(drone2.path_object.path_dict.keys()))
    #                 # Use the expected time at half edge
    #                 t2 = (drone2_t_stamps[n - 1] + drone2_t_stamps[n]) / 2
    #                 delta1 = model.vertical_protection / drone1.vertical_speed
    #                 delta2 = model.protection_area / get_drone_speed_after_node(model, drone2, graph, graph_dual,
    #                                                                             drone2.path_object.path[n - 1])
    #                 conflict = (k1, k2, t1, t2, delta1, delta2)
    #                 if check_if_conflict_is_possible(conflict, model):
    #                     descent_horiz_list.append(conflict)
    # ####
    # # Verti/verti nodes
    # # print("Check for verti/verti shared nodes")
    # # TODO remove duplicates
    # for i in range(len(model.droneList)):
    #     drone1 = model.droneList[i]
    #     for j in range(i + 1, len(model.droneList)):
    #         drone2 = model.droneList[j]
    #         if drone1.flight_number == drone2.flight_number:
    #             continue
    #         if drone1.dep == drone2.dep:
    #             for k1 in drone_trajectories_dict[drone1.flight_number]:
    #                 for k2 in drone_trajectories_dict[drone2.flight_number]:
    #                     # todo speed exacte en f de distance du next node si virage
    #                     conflict = (k1, k2, drone1.dep_time, drone2.dep_time, temps_sep_vertiport, temps_sep_vertiport)
    #                     if check_if_conflict_is_possible(conflict, model):
    #                         climb_climb_list.append(conflict)
    #         if drone1.arr == drone2.arr:
    #             for k1 in drone_trajectories_dict[drone1.flight_number]:
    #                 for k2 in drone_trajectories_dict[drone2.flight_number]:
    #                     drone2_arr_time = max(drone_trajectories_dict[drone2.flight_number][k2][0].path_dict.keys())
    #                     drone1_arr_time = max(drone_trajectories_dict[drone1.flight_number][k1][0].path_dict.keys())
    #                     conflict = (k1, k2, drone1_arr_time, drone2_arr_time, temps_sep_vertiport, temps_sep_vertiport)
    #                     if check_if_conflict_is_possible(conflict, model):
    #                         descent_descent_list.append(conflict)

    # print("Nb pt conflits :")
    # print(len(horizontal_shared_nodes_list), len(climb_horiz_list), len(descent_horiz_list), len(climb_climb_list), len(descent_descent_list))
    # return horizontal_shared_nodes_list, climb_horiz_list, descent_horiz_list, climb_climb_list, descent_descent_list


def remove_impossible_conflicts(conflict_nodes_list, model):
    # print("Removing impossible conflicts")
    # Remove impossible conflict nodes
    # A conflict can't occur if abs(t1 - t2) > delay max + delta FL max + sep 12 ou 21
    protection_area, nb_FL, delay_max, FL_sep, FL_min, temps_sep_vertiport = model.generation_params
    max_delta_fl = nb_FL - 1
    to_be_removed = []
    for index, pt in enumerate(conflict_nodes_list):
        sep12 = pt[4]
        sep21 = pt[5]
        max_delta = max_delta_fl * FL_sep / model.droneList[0].vertical_speed + delay_max
        interval1 = [pt[2] - max_delta - sep12, pt[2] + max_delta + sep12]
        interval2 = [pt[3] - max_delta - sep12, pt[3] + max_delta + sep21]
        if not (interval1[0] <= pt[3] <= interval1[1] or interval2[0] <= pt[2] <= interval2[1]):
            to_be_removed.append(index)
    to_be_removed.reverse()
    for index in to_be_removed:
        conflict_nodes_list.pop(index)


def check_if_conflict_is_possible(conflict, model):
    protection_area, nb_FL, delay_max, FL_sep, FL_min, temps_sep_vertiport = model.generation_params
    max_delta_fl = nb_FL - 1
    sep12 = conflict[4]
    sep21 = conflict[5]
    max_delta = max_delta_fl * FL_sep / model.droneList[0].vertical_speed + delay_max
    interval1 = [conflict[2] - max_delta - sep12, conflict[2] + max_delta + sep12]
    interval2 = [conflict[3] - max_delta - sep12, conflict[3] + max_delta + sep21]
    if not (interval1[0] <= conflict[3] <= interval1[1] or interval2[0] <= conflict[2] <= interval2[1]):
        return False
    else:
        return True


#
# def main():
#     ####
#     # Init
#     t_start = time.time()
#     print("Initialising graph")
#
#     load_dual = True
#     if not load_dual:
#         graph, graph_dual = md.init_graphs(graph_file_path)
#         # print("Type of node :", type(list(graph_dual.nodes)[0]), list(graph_dual.nodes)[0])
#         # print("Type of edge :", type(list(graph_dual.edges)[0]), list(graph_dual.edges)[0])
#         networkx.write_graphml(graph_dual, dual_graph_path)
#     else:
#         graph, graph_dual = md.init_graphs(graph_file_path, dual_path=dual_graph_path)
#         # print("Type of node :", type(list(graph_dual.nodes)[0]), list(graph_dual.nodes)[0])
#         # print("Type of edge :", type(list(graph_dual.edges)[0]), list(graph_dual.edges)[0])
#
#     raw_graph = osmnx.load_graphml(graph_file_path)
#     print("Done in ", time.time() - t_start)
#
#     t_start = time.time()
#     print("Initialize model")
#     model = md.init_model(graph, graph_dual, drone_list_file_path, protection_area)
#     print("Model initialized in ", time.time() - t_start)
#     print("Total number of drones :", len(model.droneList))
#     drone_trajectories_dict = dict()
#     t_start = time.time()
#
#     ####
#     # Generate
#     nb_flights_to_process = 100000
#     print("Generating trajectories")
#
#     # Have the trajectory pass by multiple points close to the shortest one
#     multiple_point_bool = True
#     if multiple_point_bool:
#         for drone in model.droneList[:nb_flights_to_process]:
#             # print("Generating")
#             points_to_explore_from_shortest = generate_points_from_shortest_path(model, drone, graph, 2, 3)  # 2 3
#             # print("Trajectories")
#             drone_trajectories_dict[drone.flight_number] = generate_multiple_point_trajectories(drone, graph, points_to_explore_from_shortest, model)
#             # print("Done")
#
#     # Have the trajectory pass by one specific point
#     one_point_bool = False
#     if one_point_bool:
#         points_to_explore_for_one = generate_points_to_pass_by_for_one_point(graph, 5)
#         min_number_of_trajectories = 10
#         for drone in model.droneList[:nb_flights_to_process]:
#             drone_trajectories_dict[drone.flight_number] = generate_one_point_trajectories(drone, graph, graph_dual, points_to_explore_for_one, model)
#             min_number_of_trajectories = min(min_number_of_trajectories, len(drone_trajectories_dict[drone.flight_number]))
#
#     # Display
#     display_traj = False
#     if display_traj:
#         for traj in drone_trajectories_dict["D1"]:
#             for node in graph.nodes:
#                 plt.scatter(graph.nodes[node]["x"], graph.nodes[node]["y"], color='blue')
#             x = [graph.nodes[node]["x"] for node in traj]
#             y = [graph.nodes[node]["y"] for node in traj]
#             plt.plot(x, y, marker='*', color='red')
#             plt.show()
#
#     # Keep only certain number of traj for each drone and add an id
#     new_dict = dict()
#     number_of_traj_to_keep = 5
#     id_trajectory = 0
#     for drone_flight_number in drone_trajectories_dict:
#         new_dict[drone_flight_number] = dict()
#         for i in range(min(number_of_traj_to_keep, len(drone_trajectories_dict[drone_flight_number])-1)):
#             new_dict[drone_flight_number][id_trajectory] = [drone_trajectories_dict[drone_flight_number][i]]
#             id_trajectory += 1
#     drone_trajectories_dict = new_dict
#     print("Trajectories generated in :", time.time() - t_start)
#
#     ####
#     # Metrics (length, compatibility)
#     # For each trajectory determine its total travel time and add the path_object
#     trajectories_to_fn_dict = dict()
#     for drone_flight_number in drone_trajectories_dict:
#         for id_trajectory in drone_trajectories_dict[drone_flight_number]:
#             current_drone = None
#             for drone in model.droneList:
#                 if drone.flight_number == drone_flight_number:
#                     current_drone = drone
#                     break
#             path = Path.Path(current_drone.dep_time, [])
#             path.set_path(drone_trajectories_dict[drone_flight_number][id_trajectory][0], graph, graph_dual, current_drone)
#             total_time = max(list(path.path_dict.keys()))
#             drone_trajectories_dict[drone_flight_number][id_trajectory].append(total_time)
#             drone_trajectories_dict[drone_flight_number][id_trajectory].append(path)
#             trajectories_to_fn_dict[id_trajectory] = drone_flight_number
#
#     display_traj2 = False
#     if display_traj2:
#         for drone_flight_number in drone_trajectories_dict:
#             # print(drone_flight_number)
#             for id_trajectory in drone_trajectories_dict[drone_flight_number]:
#                 traj = drone_trajectories_dict[drone_flight_number][id_trajectory][0]
#                 x = [graph.nodes[node]["x"] for node in graph.nodes]
#                 y = [graph.nodes[node]["y"] for node in graph.nodes]
#                 # for node in graph.nodes:
#                 plt.scatter(x, y, color='grey')
#                 x = [graph.nodes[node]["x"] for node in traj]
#                 y = [graph.nodes[node]["y"] for node in traj]
#                 plt.plot(x, y, marker='*', color='red')
#                 plt.show()
#     display_best_traj = False
#     if display_best_traj:
#         for drone_flight_number in drone_trajectories_dict:
#             for drone in model.droneList:
#                 if drone.flight_number == drone_flight_number:
#                     current_drone = drone
#             id = list(drone_trajectories_dict[drone_flight_number].keys())[0]
#             traj = drone_trajectories_dict[drone_flight_number][id][0]
#             x = [graph.nodes[node]["x"] for node in graph.nodes]
#             y = [graph.nodes[node]["y"] for node in graph.nodes]
#             # for node in graph.nodes:
#             plt.scatter(x, y, color='grey')
#             x = [graph.nodes[node]["x"] for node in traj]
#             y = [graph.nodes[node]["y"] for node in traj]
#             plt.plot(x, y, marker='*', color='red')
#             plt.plot(current_drone.arrival_vertiport[0], current_drone.arrival_vertiport[1], marker='o', color='purple')
#             plt.plot(current_drone.departure_vertiport[0], current_drone.departure_vertiport[1], marker='o', color='pink')
#             plt.show()
#
#     size = 0
#     for drone_flight_number in drone_trajectories_dict:
#         size += len(drone_trajectories_dict[drone_flight_number])
#     print("Total nb of trajectories :", size)
#
#     #####
#     # Check for shared nodes between trajectories
#     print("Check for shared nodes on horizontal route")
#     # For each trajectory, list of shared nodes
#     horizontal_shared_nodes_list = []
#     for i in range(size):
#         for j in range(i+1, size):
#             flight_number1 = trajectories_to_fn_dict[i]
#             flight_number2 = trajectories_to_fn_dict[j]
#             if flight_number1 == flight_number2:
#                 continue
#             drone1 = return_drone_from_flight_number(model, flight_number1)
#             drone2 = return_drone_from_flight_number(model, flight_number2)
#             drone1.path_object = drone_trajectories_dict[flight_number1][i][2]
#             drone2.path_object = drone_trajectories_dict[flight_number2][j][2]
#             traj1 = i + 1
#             traj2 = j + 1
#             # Nodes that both trajectories use
#             for node in drone1.path_object.path:
#                 if node in drone2.path_object.path:
#                     t1 = None
#                     for t in drone1.path_object.path_dict:
#                         if drone1.path_object.path_dict[t] == node:
#                             t1 = t
#                             break
#                     t2 = None
#                     for t in drone2.path_object.path_dict:
#                         if drone2.path_object.path_dict[t] == node:
#                             t2 = t
#                             break
#                     v1 = get_drone_speed_after_node(drone1, graph, graph_dual, node)
#                     v2 = get_drone_speed_after_node(drone2, graph, graph_dual, node)
#
#                     # drone_shared_nodes_tab[i][j].append((t1, t2, protection_area/v1, protection_area/v2))
#                     horizontal_shared_nodes_list.append((traj1, traj2, t1, t2, protection_area/v1, protection_area/v2))
#
#                     # TODO pour les conflit sur edge je modifie ou j'ajoute nouvelle contrainte ? Il faut choisir le max ou ajouter ?
#                     t_stamps_d1 = sorted(list(drone1.path_object.path_dict.keys()))
#                     t_stamps_d2 = sorted(list(drone2.path_object.path_dict.keys()))
#                     # Same way
#                     if t1 != t_stamps_d1[-1] and t2 != t_stamps_d2[-1]:
#                         d1_t_next_node = t_stamps_d1[t_stamps_d1.index(t1)+1]
#                         d2_t_next_node = t_stamps_d2[t_stamps_d2.index(t2)+1]
#                         d1_next_node = drone1.path_object.path_dict[d1_t_next_node]
#                         d2_next_node = drone2.path_object.path_dict[d2_t_next_node]
#                         # same way
#                         if d1_next_node == d2_next_node:
#                             if t1 < t2:
#                                 # If drone1 goes first we need to make sure the delta2 is enough for drone2 not to catch up
#                                 # (t1-d1_t_next_node)-(t2-d2_t_next_node) > 0 if 2 is faster
#                                 delta_2 = max(protection_area/v1, (t1-d1_t_next_node)-(t2-d2_t_next_node))
#                                 horizontal_shared_nodes_list.append((traj1, traj2, t1, t2, protection_area/v1, delta_2))
#                             if t2 < t1:
#                                 # If drone2 goes first we need to make sure the delta1 is enough for drone2 not to catch up
#                                 delta_1 = max(protection_area/v2, (t2-d2_t_next_node)-(t1-d1_t_next_node))
#                                 horizontal_shared_nodes_list.append((traj1, traj2, t1, t2, delta_1, protection_area/v2))
#                     #Opposite ways
#                     # If D1 goes first
#                     if t1 != t_stamps_d1[-1] and t2 != t_stamps_d2[0]:
#                         if t1 < t2:
#                             d1_t_next_node = t_stamps_d1[t_stamps_d1.index(t1)+1]
#                             d2_t_prev_node = t_stamps_d2[t_stamps_d2.index(t2)-1]
#                             d1_next_node = drone1.path_object.path_dict[d1_t_next_node]
#                             d2_prev_node = drone2.path_object.path_dict[d2_t_prev_node]
#                             if d1_next_node == d2_prev_node:
#                                 delta_1 = (t2 - d2_t_prev_node) + (d1_t_next_node - t1)
#                                 horizontal_shared_nodes_list.append((traj1, traj2, t1, t2, delta_1, protection_area / v2))
#                     # if D2 goes first
#                     if t2 != t_stamps_d2[-1] and t1 != t_stamps_d1[0]:
#                         if t2 < t1:
#                             d2_t_next_node = t_stamps_d2[t_stamps_d2.index(t2)+1]
#                             d1_t_prev_node = t_stamps_d1[t_stamps_d1.index(t1)-1]
#                             d2_next_node = drone2.path_object.path_dict[d2_t_next_node]
#                             d1_prev_node = drone1.path_object.path_dict[d1_t_prev_node]
#                             if d2_next_node == d1_prev_node:
#                                 delta_2 = (t1 - d1_t_prev_node) + (d2_t_next_node - t2)
#                                 horizontal_shared_nodes_list.append((traj1, traj2, t1, t2, protection_area / v1, delta_2))
#
#     ####
#     # Get all drones vertiports (arrival and departure)
#     print("Searching for departure and arrival edges for each drone")
#     dep_edge_dict, arrival_edge_dict = get_all_dep_and_arr_edges(model, raw_graph)
#
#     ####
#     # Horizontal/Vertical shared nodes
#     # TODO est ce qu'on veut les deux i,j j,i ou que un des deux suffit
#     print("Check for vertically shared nodes with horizontal route")
#     climb_horiz_list = []
#     descent_horiz_list = []
#     for i in range(size):
#         # print("D" + str(1 + (i // number_of_traj_to_keep)))
#         flight_number1 = trajectories_to_fn_dict[i]
#         drone1 = return_drone_from_flight_number(model, flight_number1)
#         drone1.path_object = drone_trajectories_dict[flight_number1][i][2]
#         dep_edge = dep_edge_dict[drone1.flight_number]
#         arr_edge = arrival_edge_dict[drone1.flight_number]
#         for j in range(size):
#             flight_number2 = trajectories_to_fn_dict[j]
#             if flight_number1 == flight_number2:
#                 continue
#             drone2 = return_drone_from_flight_number(model, flight_number2)
#             drone2.path_object = drone_trajectories_dict[flight_number2][j][2]
#             # Find the edges the drone start and end on
#             # Check if either is part of the other drone path
#             # print(dep_edge)
#             # TODO Faire les delta t
#             for n in range(1, len(drone2.path_object.path)):
#                 if dep_edge == (drone2.path_object.path[n-1], drone2.path_object.path[n]) or (dep_edge[1], dep_edge[0]) == (drone2.path_object.path[n-1], drone2.path_object.path[n]):
#                     # k1, k2, t1, t2, delta1, delta2
#                     k1, k2 = i+1, j+1
#                     t1 = drone1.dep_time
#                     drone2_t_stamps = sorted(list(drone2.path_object.path_dict.keys()))
#                     # Use the expected time at half edge
#                     t2 = (drone2_t_stamps[n-1] + drone2_t_stamps[n]) / 2
#                     delta1 = model.vertical_protection / drone1.vertical_speed
#                     delta2 = model.protection_area / get_drone_speed_after_node(drone2, graph, graph_dual, drone2.path_object.path[n-1])
#                     climb_horiz_list.append((k1, k2, t1, t2, delta1, delta2))
#                 if arr_edge == (drone2.path_object.path[n-1], drone2.path_object.path[n]) or (arr_edge[1], arr_edge[0]) == (drone2.path_object.path[n-1], drone2.path_object.path[n]):
#                     # k1, k2, t1, t2, delta1, delta2
#                     k1, k2 = i+1, j+1
#                     drone1_t_stamps = sorted(list(drone1.path_object.path_dict.keys()))
#                     t1 = drone1_t_stamps[-1]
#                     drone2_t_stamps = sorted(list(drone2.path_object.path_dict.keys()))
#                     # Use the expected time at half edge
#                     t2 = (drone2_t_stamps[n-1] + drone2_t_stamps[n]) / 2
#                     delta1 = model.vertical_protection / drone1.vertical_speed
#                     delta2 = model.protection_area / get_drone_speed_after_node(drone2, graph, graph_dual, drone2.path_object.path[n-1])
#                     descent_horiz_list.append((k1, k2, t1, t2, delta1, delta2))
#     # print("Verti/Horiz shared nodes\n", vertically_shared_edges_list)
#     ####
#     # Verti/verti nodes
#     print("Check for verti/verti shared nodes")
#     climb_climb_list = []
#     descent_descent_list = []
#     # TODO remove duplicates
#     for i in range(len(model.droneList)):
#         drone1 = model.droneList[i]
#         for j in range(i+1, len(model.droneList)):
#             drone2 = model.droneList[j]
#             if drone1.flight_number == drone2.flight_number:
#                 continue
#             if drone1.dep == drone2.dep:
#                 for k1 in drone_trajectories_dict[drone1.flight_number]:
#                     for k2 in drone_trajectories_dict[drone2.flight_number]:
#                         #todo speed exacte en f de distance du next node si virage
#                         climb_climb_list.append((k1+1,k2+1,drone1.dep_time, drone2.dep_time, temps_sep_vertiport, temps_sep_vertiport))
#             # if drone1.dep == drone2.arr:
#             #     # Si drone descendant arrive avant on attend qu'il se soit eloigne de dist secu verti, sinon horizontale
#             #     for k1 in drone_trajectories_dict[drone1.flight_number]:
#             #         for k2 in drone_trajectories_dict[drone2.flight_number]:
#             #             drone2_arr_time = max(drone_trajectories_dict[drone2.flight_number][k2][2].path_dict.keys())
#             #             shared_vertiports.append((k1,k2,drone1.dep_time, drone2_arr_time, model.protection_area/drone1.cruise_speed, model.vertical_protection/drone2.vertical_speed))
#             # if drone1.arr == drone2.dep:
#             #     for k1 in drone_trajectories_dict[drone1.flight_number]:
#             #         for k2 in drone_trajectories_dict[drone2.flight_number]:
#             #             drone1_arr_time = max(drone_trajectories_dict[drone1.flight_number][k1][2].path_dict.keys())
#             #             shared_vertiports.append((k1,k2,drone1_arr_time, drone2.dep_time, model.vertical_protection/drone1.vertical_speed, model.protection_area/drone2.cruise_speed))
#             if drone1.arr == drone2.arr:
#                 for k1 in drone_trajectories_dict[drone1.flight_number]:
#                     for k2 in drone_trajectories_dict[drone2.flight_number]:
#                         drone2_arr_time = max(drone_trajectories_dict[drone2.flight_number][k2][2].path_dict.keys())
#                         drone1_arr_time = max(drone_trajectories_dict[drone1.flight_number][k1][2].path_dict.keys())
#                         descent_descent_list.append((k1+1, k2+1, drone1_arr_time, drone2_arr_time, temps_sep_vertiport, temps_sep_vertiport))
#
#     ####
#     print("Removing impossible conflicts")
#     # Remove impossible conflict nodes
#     # A conflict can't occur if abs(t1 - t2) > delay max + delta FL max + sep 12 ou 21
#     to_be_removed = []
#     for index, pt in enumerate(horizontal_shared_nodes_list):
#         max_delta = max_delta_fl * FL_sep / model.droneList[0].vertical_speed + delay_max
#         interval1 = [pt[2] - max_delta, pt[2] + max_delta]
#         interval2 = [pt[3] - max_delta, pt[3] + max_delta]
#         if not (interval1[0] <= pt[3] <= interval1[1] or interval2[0] <= pt[2] <= interval2[1]):
#             to_be_removed.append(index)
#     to_be_removed.reverse()
#     for index in to_be_removed:
#         horizontal_shared_nodes_list.pop(index)
#
#     #TODO remove et faire verti/hori et verti/verti
#
#     print("Write PLNE outputs")
#     with open("./PLNE OUTPUT/output.dat", 'w') as file:
#         file.write("param nbflights := " + str(len(drone_trajectories_dict)) + ";\n")
#         file.write("\nparam nbFL := " + str(nb_FL) + ";\n")
#         file.write("param maxDelay := " + str(delay_max) + ";\n")
#         file.write("param nbTrajs := " + str(sum([len(drone_trajectories_dict[d]) for d in drone_trajectories_dict])) + ";\n")
#         file.write("param nbPtHor := " + str(len(horizontal_shared_nodes_list)) + ";\n")
#         file.write("param nbPtClmb := " + str(len(climb_horiz_list)) + ";\n")
#         file.write("param nbPtDesc := " + str(len(descent_horiz_list)) + ";\n")
#         file.write("param nbPtDep := " + str(len(climb_climb_list)) + ";\n")
#         file.write("param nbPtArr := " + str(len(descent_descent_list)) + ";\n")
#         file.write("\n")
#         file.write("param: d mon_vol :=")
#         for drone in drone_trajectories_dict:
#             for traj_id in drone_trajectories_dict[drone]:
#                 # print(drone_trajectories_dict[drone][traj_id])
#                 idx = get_drone_index_in_model_list(drone, model)
#                 file.write("\n" + str(traj_id + 1) + " " + str(int(drone_trajectories_dict[drone][traj_id][1])) + " " + str(idx+1))
#         file.write(";\n\n")
#         file.write("param: k1 k2 t1 t2 sep12 sep21 :=")
#         for index, traj in enumerate(horizontal_shared_nodes_list):
#             file.write("\n")
#             file.write(str(index+1))
#             file.write(" " + str(traj[0]) + " " + str(traj[1]))
#             file.write(" " + str(round(traj[2], 1)) + " " + str(round(traj[3], 1)))
#             file.write(" " + str(round(traj[4], 1)) + " " + str(round(traj[5], 1)))
#         last_index = len(horizontal_shared_nodes_list)
#         for index, traj in enumerate(climb_horiz_list):
#             file.write("\n")
#             file.write(str(last_index + index + 1))
#             file.write(" " + str(traj[0]) + " " + str(traj[1]))
#             file.write(" " + str(round(traj[2], 1)) + " " + str(round(traj[3], 1)))
#             file.write(" " + str(round(traj[4], 1)) + " " + str(round(traj[5], 1)))
#         last_index += len(climb_horiz_list)
#         for index, traj in enumerate(descent_horiz_list):
#             file.write("\n")
#             file.write(str(last_index + index + 1))
#             file.write(" " + str(traj[0]) + " " + str(traj[1]))
#             file.write(" " + str(round(traj[2], 1)) + " " + str(round(traj[3], 1)))
#             file.write(" " + str(round(traj[4], 1)) + " " + str(round(traj[5], 1)))
#         last_index += len(descent_horiz_list)
#         for index, traj in enumerate(climb_climb_list):
#             file.write("\n")
#             file.write(str(last_index + index + 1))
#             file.write(" " + str(traj[0]) + " " + str(traj[1]))
#             file.write(" " + str(round(traj[2], 1)) + " " + str(round(traj[3], 1)))
#             file.write(" " + str(round(traj[4], 1)) + " " + str(round(traj[5], 1)))
#         last_index += len(climb_climb_list)
#         for index, traj in enumerate(descent_descent_list):
#             file.write("\n")
#             file.write(str(last_index + index + 1))
#             file.write(" " + str(traj[0]) + " " + str(traj[1]))
#             file.write(" " + str(round(traj[2], 1)) + " " + str(round(traj[3], 1)))
#             file.write(" " + str(round(traj[4], 1)) + " " + str(round(traj[5], 1)))
#         last_index += len(descent_descent_list)
#         file.write(";\n")
#     print("Write FN trajectories match")
#     with open("./PLNE OUTPUT/FN_traj.dat", 'w+') as file:
#         for drone in drone_trajectories_dict:
#             for traj_id in drone_trajectories_dict[drone]:
#                 file.write(str(drone) + " " + str(traj_id) + " \n")
#                 for _s in drone_trajectories_dict[drone][traj_id][0]:
#                     file.write(_s + " ")
#                 file.write("\n")
#
#     ####
#     # Check for conflicts
#     check_for_conflict_bool = False
#     if check_for_conflict_bool:
#         print("Check for conflicts")
#         # For each trajectory determine if there is a conflict with another
#         size = 0
#         for drone_flight_number in drone_trajectories_dict:
#             size += len(drone_trajectories_dict[drone_flight_number])
#         # print(size)
#         compatibility_matrix = np.zeros((size, size))
#         # 1 if flight i and j are compatible, 0 if there is a conflict
#         for i in range(size):
#             # print("D" + str(1 + (i // number_of_traj_to_keep)))
#             for j in range(i+1, size):
#                 flight_number1 = "D" + str(1 + (i // number_of_traj_to_keep))
#                 flight_number2 = "D" + str(1 + (j // number_of_traj_to_keep))
#                 if flight_number1 == flight_number2:
#                     compatibility_matrix[i][j] = 0
#                     continue
#                 drone1 = return_drone_from_flight_number(model, flight_number1)
#                 drone1.path_object = drone_trajectories_dict[flight_number1][i][2]
#                 drone2 = return_drone_from_flight_number(model, flight_number2)
#                 drone2.path_object = drone_trajectories_dict[flight_number2][j][2]
#
#                 if model.find_conflicts_on_specified_drones([drone1, drone2]) == []:
#                     compatibility_matrix[i][j] = 1
#                 else:
#                     compatibility_matrix[i][j] = 0
#
#         # Make the matrix symetrical :
#         for i in range(size):
#             for j in range(i):
#                 compatibility_matrix[i, j] = compatibility_matrix[j, i]
#     print("Total time = ", time.time() - t_start)


def get_all_dep_and_arr_edges(model, raw_graph):
    dep_edge_dict = dict()
    arrival_edge_dict = dict()
    # count = 0
    for drone in model.droneList:
        # print(count)
        # count += 1
        hash_nodes, hash_edges, min_x, min_y, x_step, y_step, resolution = model.hash_map
        x_dep, y_dep = drone.departure_vertiport[0], drone.departure_vertiport[1]
        list_of_potential_closest_edges = tools.find_list_of_closest_with_hash(x_dep, y_dep, hash_edges, min_x, min_y,
                                                                               x_step, y_step, resolution)
        dep = tools.find_closest_edge_in_list(x_dep, y_dep, list_of_potential_closest_edges, model.graph)
        x_arr, y_arr = drone.arrival_vertiport[0], drone.arrival_vertiport[1]
        list_of_potential_closest_edges = tools.find_list_of_closest_with_hash(x_arr, y_arr, hash_edges, min_x, min_y,
                                                                               x_step, y_step, resolution)
        arr = tools.find_closest_edge_in_list(x_arr, y_arr, list_of_potential_closest_edges, model.graph)
        dep_edge_dict[drone.flight_number] = dep
        arrival_edge_dict[drone.flight_number] = arr
    # with warnings.catch_warnings():
    #     warnings.simplefilter("ignore")
    #     dep_edge = osmnx.get_nearest_edge(raw_graph, (drone.departure_vertiport[1], drone.departure_vertiport[0]))[0:2]
    #     dep_edge = (str(dep_edge[0]), str(dep_edge[1]))
    #     dep_edge_dict[drone.flight_number] = dep_edge
    #     arr_edge = osmnx.get_nearest_edge(raw_graph, (drone.arrival_vertiport[1], drone.arrival_vertiport[0]))[0:2]
    #     arr_edge = (str(arr_edge[0]), str(arr_edge[1]))
    #     arrival_edge_dict[drone.flight_number] = arr_edge
    return dep_edge_dict, arrival_edge_dict


def generate_parallel_trajectories(drone, graph, model, step, dist, number_to_generate):
    trajectories = []
    # trajectories_to_fn, trajectories_to_duration, trajectories_to_path, fn_order =
    drone_dep_dual = ("S" + drone.dep, drone.dep)
    drone_arr_dual = (drone.arr, drone.arr + "T")
    # print(drone_dep_dual, drone_arr_dual)
    shortest_path = a2.astar_dual(model, drone_dep_dual, drone_arr_dual, drone, drone.dep_time)
    # print(shortest_path.path)
    trajectories.append(shortest_path.path)

    geodesic = pyproj.Geod(ellps='WGS84')
    x_dep, y_dep = graph.nodes[drone.dep]["x"], graph.nodes[drone.dep]["y"]
    x_arr, y_arr = graph.nodes[drone.arr]["x"], graph.nodes[drone.arr]["y"]

    heading, _back_azimuth1, _distance = geodesic.inv(x_dep, y_dep, x_arr, y_arr)
    nodes_to_deviate_from = []
    for i in range(1, step + 1):
        nodes_to_deviate_from.append(shortest_path.path[i * (len(shortest_path.path) // (step + 1))])
    # print("path and nodes to deviate from",  shortest_path.path, nodes_to_deviate_from)
    count = 0
    tries = 0
    trajectory = []
    # plt.show()
    while len(trajectories) < number_to_generate and tries < 20:
        tries += 1
        if tries == 20:
            pass
        # print("COULDN'T PRODUCE ENOUGH ALTERNATIVE TRAJS")
        nodes_to_visit = []

        if count % 2 == 1:
            new_heading = (heading + 90) % 360
        else:
            new_heading = (heading - 90) % 360

        for node in nodes_to_deviate_from:
            # generer un noeud dans une direction
            hash_nodes, hash_edges, min_x, min_y, x_step, y_step, resolution = model.hash_map
            pt = (graph.nodes[node]["x"], graph.nodes[node]["y"])
            north_factor = math.cos(new_heading)
            east_factor = math.sin(new_heading)
            new_pt = tools.m_displacement_to_lat_lon(pt, north_factor * dist * (1 + count // 2),
                                                     east_factor * dist * (1 + count // 2))
            new_x = new_pt[0]
            new_y = new_pt[1]
            new_node = tools.find_closest_node_with_hash(new_x, new_y, hash_nodes, min_x, min_y, x_step, y_step,
                                                         resolution, model.graph)
            nodes_to_visit.append(new_node)
        # get closest node

        drone_dep_dual = ("S" + drone.dep, drone.dep)
        drone_arr_dual = (nodes_to_visit[0], nodes_to_visit[0] + "T")
        trajectory = a2.astar_dual(model, drone_dep_dual, drone_arr_dual, drone, drone.dep_time).path
        for i in range(len(nodes_to_visit) - 1):
            node = nodes_to_visit[i]
            next_node = nodes_to_visit[i + 1]
            drone_dep_dual = ("S" + node, node)
            drone_arr_dual = (next_node, next_node + "T")
            _path = a2.astar_dual(model, drone_dep_dual, drone_arr_dual, drone, drone.dep_time).path[1:]
            trajectory += _path
        drone_dep_dual = ("S" + nodes_to_visit[-1], nodes_to_visit[-1])
        drone_arr_dual = (drone.arr, drone.arr + "T")
        trajectory += a2.astar_dual(model, drone_dep_dual, drone_arr_dual, drone, drone.dep_time).path[1:]
        trajectory_ok = True

        if len(set(trajectory)) != len(trajectory):
            # print("trajectory has doublons")
            # print(trajectory)
            # for i in range(len(trajectory)):
            #     if trajectory[i] in trajectory[i+1:]:
            # print(trajectory[i])
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
                # print("traj still has doublons")
                # print(trajectory)
                # for i in range(len(trajectory)):
                #     if trajectory[i] in trajectory[i+1:]:
                #         print(trajectory[i])
                trajectory_ok = False
            if not check_traj_is_possible(graph, trajectory):
                print("TRAJ impossible")
                trajectory_ok = False

        # tools.scatter_graph(graph)
        # tools.plot_traj(shortest_path.path, graph)
        # tools.plot_traj(nodes_to_visit, graph, color='purple', marker='*')
        # tools.plot_traj(trajectory, graph, color='blue', marker='x')
        # plt.show()

        if trajectory not in trajectories and trajectory_ok:
            trajectories.append(trajectory)
        count += 1

    return trajectories


def get_drone_index_in_model_list(drone_to_search_fn, model):
    index = 0
    for drone in model.droneList:
        if drone_to_search_fn == drone.flight_number:
            return index
        index += 1


def get_drone_speed_after_node(model, drone, graph, graph_dual, node):
    # Find the other drone speed when passing the entry node of the edge
    drone_speed_after_node = None
    drone_path = drone.path_object
    node_index = drone_path.path.index(node)
    node_minus1, node_plus1, node_plus2 = None, None, None
    if node_index < len(drone_path.path) - 1:
        node_plus1 = drone_path.path[node_index + 1]
    if node_index < len(drone_path.path) - 2:
        node_plus2 = drone_path.path[node_index + 2]
    if node_index >= 1:
        node_minus1 = drone_path.path[node_index - 1]
    # Was the last node a turn point
    if node_minus1 is not None and node_plus1 is not None:
        if graph_dual.edges[(node_minus1, node), (node, node_plus1)]["is_turn"]:
            drone_speed_after_node = drone.turn_speed
    # Is the next node a turning point and is it far enough ?
    if node_plus1 is not None and node_plus2 is not None:
        if graph_dual.edges[(node, node_plus1), (node_plus1, node_plus2)]["is_turn"]:
            # Check that the distance between the 2 nodes is sufficient
            length = graph.edges[node, node_plus1]['length']
            # Dist need to be more than braking distance + safety
            if length < model.protection_area + drone.braking_distance:
                drone_speed_after_node = drone.turn_speed

    if drone_speed_after_node is None:
        drone_speed_after_node = drone.cruise_speed
    return drone_speed_after_node


def return_drone_from_flight_number(model, flight_number):
    return model.total_drone_dict[flight_number]
    # for drone in model.total_drone_list:
    #     if drone.flight_number == flight_number:
    #         return drone
    # print("Drone not in list", flight_number, [d.flight_number for d in model.droneList])
    # raise Exception


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


def check_traj_is_possible(graph, traj):
    for index in range(len(traj) - 1):
        if (traj[index], traj[index + 1]) not in graph.edges and (traj[index + 1], traj[index]) not in graph.edges:
            return False
    return True

#
# if __name__ == "__main__":
#     main()
