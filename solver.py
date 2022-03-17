import math
import generate_trajectories
import Model as md
import osmnx
import Param
import PLNE
import tools
import BlueskySCNTools
import networkx
import csv
import Drone

graph_file_path = "graph_files/total_graph_200m.graphml"
dual_graph_path = "graph_files/dual_total_graph_200m_with_geofences_no_demi_tour.graphml"
# dual_graph_path = None
save_path = "graph_files/dual_total_graph_200m_with_geofences.graphml"
drone_list_file_path = 'graph_files/Intentions/M2_final_flight_intentions/flight_intentions/Flight_intention_very_low_50_1.csv'
output_scenario = "PLNE OUTPUT/scenario.scn"
output_scenario_with_RTA = "PLNE OUTPUT/scenario_with_RTA.scn"

T_MAX_OPTIM = 50
MIP_GAP = 0.3
ms_to_knots = 1.94384
m_to_feet = 3.28084
drone_delete_dist = 0.0026997840172786176 * 3

# TODO a modifier ensuite
TIME_MARGIN = 0
rta_time_deviation = 0


def solve_with_time_segmentation():
    """"SOLVE the problem by using time segmentation.
    Dynamic geofence flights are sorted to be computed first and are always included in resolution"""
    print("-- INIT")
    print("Initialising graph")
    graph, graph_dual = md.init_graphs(graph_file_path, dual_path=dual_graph_path)
    if dual_graph_path is None:
        print("Saving dual graph")
        networkx.write_graphml(graph_dual, save_path)
    raw_graph = osmnx.load_graphml(graph_file_path)
    resolution = 100
    graph_hash = tools.hash_graph(graph, resolution)
    print("Graphs initialised")
    # No specified time, to load all drones at first
    model = md.init_model(graph, graph_dual, drone_list_file_path, graph_hash)
    # Dict used to store the drones and give them to the model.
    fn_to_drones_dict = dict()
    for drone in model.droneList:
        fn_to_drones_dict[drone.flight_number] = drone
    model.total_drone_dict = fn_to_drones_dict
    drones_with_dynamic_fences = get_drones_with_dynamic_geofences_flight_number(drone_list_file_path)
    print("TOTAL NB DRONES :", len(model.droneList))
    print("-- INITIALISED")

    #####
    # SET ALL FLIGHT LEVELS FOR AVAILABLE FLIGHTS AT T = 0
    set_model_drone_list(model, 0, math.inf)
    fixed_flight_levels = solve_flight_levels_current_model(model, graph,raw_graph,graph_dual)

    #####
    # DYNAMIC GEOFENCES DRONES
    fixed_flights_dict = dict()
    print("Generating trajectories for dynamic geofences")
    # Keep only the dynamic geofences drones
    model.droneList = []
    if len(drones_with_dynamic_fences) > 0:
        for drone_with_fence in drones_with_dynamic_fences:
            drone_fn = drone_with_fence[0]
            model.droneList.append(fn_to_drones_dict[drone_fn])
        current_param = None
        # SOLVE
        print("\n-------------------------\n--NB DRONES :", len(model.droneList))
        geofence_time_intervals = dict()
        current_fixed_flight_levels = return_current_flight_level_fixed_flights(fixed_flight_levels, model, fixed_flights_dict)
        traj_output, intersection_outputs, problem, param = solve_current_model(model, graph, raw_graph, graph_dual, current_param, fixed_flights_dict, geofence_time_intervals, current_fixed_flight_levels)
        trajectories, trajectories_to_fn, trajectories_to_duration, trajectories_to_path, fn_order = traj_output
        # Add the dynamic geofence drones to the fixed flights list
        for k in problem.param.K:
            # For each of the drone find the chosen traj, fl, delay
            if problem.x[k].x == 1:
                drone_fn = trajectories_to_fn[k]
                a = drone_fn
                fixed_flights_dict[drone_fn] = [a, k, problem.y[a].x, problem.delay[a].x]
        generate_SCN_v2(model, problem, trajectories, trajectories_to_fn, "PLNE OUTPUT/scenarioV2_" + "geofence" + ".scn")
        generate_time_stamps(model, graph)
        # Create the geofenced nodes list
        geofence_time_intervals = dict()
        for drone_fn, fixed_flight in fixed_flights_dict.items():
            activation_time = fixed_flight[3] * Param.delayStep
            activation_time += fixed_flight[2] * model.FL_sep / Drone.vertical_speed
            activation_time += trajectories_to_path[fixed_flight[1]].arr_time
            drone = fn_to_drones_dict[drone_fn]
            for node in find_nodes_in_geofence(drone.loitering_geofence, graph):
                interval = activation_time, activation_time + drone.loitering_geofence[0]
                if interval in geofence_time_intervals:
                    geofence_time_intervals[interval].append(node)
                else:
                    geofence_time_intervals[interval] = [node]
    else:
        traj_output = None
        geofence_time_intervals = dict()
        print("No geofence drones")

    #####
    # FULL RESOLUTION
    print("Starting resolution")
    # Load a ever increasing list of drone in the model, and don't forget the dynamic geofences ones
    sim_step = 600
    sim_size = 1200
    sim_time = 0
    last_departure = max([model.total_drone_dict[drone_fn].dep_time for drone_fn in model.total_drone_dict])
    print("Last departure : ", last_departure)
    while sim_time <= last_departure + sim_step:
        set_model_drone_list(model, sim_time,  sim_size)
        print("\n-------------------------\n--NB DRONES :", len(model.droneList))
        print("-- ", len(fixed_flights_dict.keys()), " Fixed flights at this time")
        current_fixed_flight_levels = return_current_flight_level_fixed_flights(fixed_flight_levels, model, fixed_flights_dict)
        traj_output, intersection_outputs, problem, param = solve_current_model(model, graph, raw_graph, graph_dual, traj_output, fixed_flights_dict, geofence_time_intervals, current_fixed_flight_levels)
        trajectories, trajectories_to_fn, trajectories_to_duration, trajectories_to_path, fn_order = traj_output
        for k in problem.param.K:
            if problem.x[k].x == 1:
                drone_fn = trajectories_to_fn[k]
                drone = model.total_drone_dict[drone_fn]
                if drone.dep_time < sim_time + sim_step and drone_fn not in fixed_flights_dict:
                    a = drone_fn
                    fixed_flights_dict[drone_fn] = [a, k, problem.y[a].x, problem.delay[a].x]
        generate_SCN_v2(model, problem, trajectories, trajectories_to_fn, "PLNE OUTPUT/scenarioV2_" + str(sim_time) + ".scn")
        sim_time += sim_step

    # Generate SCN
    generate_SCN_v2(model, problem, trajectories, trajectories_to_fn, "PLNE OUTPUT/scenarioV2_Final.scn")


def solve_current_model(model, graph, raw_graph, graph_dual, current_param, fixed_flights_dict, geofence_time_intervals, fixed_flight_levels):
    ####
    # Solve current state of the model :
    # - Generate trajectories
    print("-- Generating trajectories")
    trajectories, trajectories_to_fn, trajectories_to_duration, trajectories_to_path, fn_order = generate_trajectories.generate_trajectories(model, graph, raw_graph, graph_dual, geofence_time_intervals, current_param)
    trajectories_output = trajectories, trajectories_to_fn, trajectories_to_duration, trajectories_to_path, fn_order
    # - Generate intersection points
    print("-- Intersection points")
    h_h_list,climb_h_list,descent_h_list,c_c_list,d_d_list = generate_trajectories.generate_intersection_points(trajectories, trajectories_to_fn, trajectories_to_path, model, graph, graph_dual, raw_graph)
    intersection_outputs = h_h_list,climb_h_list,descent_h_list,c_c_list,d_d_list
    Afix, Kfix, yfix, delayfix = [], [], [], []
    for drone in fixed_flights_dict:
        Afix.append(fixed_flights_dict[drone][0])
        Kfix.append(fixed_flights_dict[drone][1])
        yfix.append(fixed_flights_dict[drone][2])
        delayfix.append(fixed_flights_dict[drone][3])
    # - Create param
    param = create_param(model, trajectories,trajectories_to_duration,h_h_list,climb_h_list,descent_h_list,c_c_list,d_d_list,fixed_flights_dict, fixed_flight_levels)
    # - Create Problem
    problem = PLNE.ProblemGlobal(param)
    problem.model.setParam("TimeLimit", T_MAX_OPTIM)
    # problem.model.setParam("MIPGap", MIP_GAP)
    # Solve
    problem.solve()
    # problem.printSolution()
    return trajectories_output, intersection_outputs, problem, param


def solve_flight_levels_current_model(model, graph, raw_graph, graph_dual):
    """Find a flight level assignation for all the drone of th model that reduce the number of conflicts"""
    # Generate only one trajectory for each drone (only the shortest)
    print("Generating shortest trajectories")
    current_param = None
    geofence_time_intervals = []
    trajectories, trajectories_to_fn, trajectories_to_duration, trajectories_to_path, fn_order = generate_trajectories.generate_trajectories(model, graph, raw_graph, graph_dual, geofence_time_intervals, current_param, number_of_trajectories=1)
    # Generate conflict matrix
    print("Computing interactions")
    interactions = generate_trajectories.generate_interaction(trajectories, trajectories_to_fn, trajectories_to_path, model, graph, graph_dual, raw_graph)
    A = trajectories.keys()
    param_level = Param.ParamLevelChoice(model, A, interactions)
    problem = PLNE.ProblemLevelChoice(param_level)
    problem.model.setParam("TimeLimit", T_MAX_OPTIM)
    problem.solve()
    # problem.printSolution()
    # Generate fixed_flight_levels_dict
    fixed_flight_levels = []
    for a in problem.param.A:
        fixed_flight_levels.append((a, problem.y[a].x))
    return fixed_flight_levels


def return_current_flight_level_fixed_flights(fixed_flight_levels, model, fixed_flights_dict):
    # Return the drones from the model.drone_list that are in the fixed_flight_list but not in fixed flight
    current_fixed_flight_levels_list = []
    fn_list = [drone.flight_number for drone in model.droneList]
    for a, fl in fixed_flight_levels:
        if a in fn_list and not a in fixed_flights_dict:
            current_fixed_flight_levels_list.append((a,fl))
    return current_fixed_flight_levels_list


def set_model_drone_list(model, sim_time, sim_size):
    """Add the drones from the total_list in the droneList depending on specified time
    Drones which are dynamic geofences drones (and should be the first of the list are added too"""
    model.droneList = []
    for drone_fn in model.total_drone_dict:
        drone = model.total_drone_dict[drone_fn]
        if drone.is_loitering_mission:
            model.droneList.append(drone)
        elif drone.dep_time <= sim_time + sim_size and not drone.deposit_time > sim_time:
            model.droneList.append(drone)


def sort_flight_intention(flight_intention_file):
    """Sort the flight intention file to have the dynamic geofences flights first and then by take off time"""
    output_path = flight_intention_file[:-4] + "_sorted.csv"
    dynamic_geofences_flights = []
    other_flights = []
    with open(flight_intention_file, newline='') as csvfile:
        reader = csv.reader(csvfile, delimiter=',', quotechar='|')
        for row in reader:
            if row[9] != '':
                dynamic_geofences_flights.append(row)
            else:
                other_flights.append(row)
    with open(output_path, 'w') as f:
        pass
    with open(output_path, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile, delimiter=',', quotechar='|')
        for row in dynamic_geofences_flights:
            writer.writerow(row)
        for row in other_flights:
            writer.writerow(row)
    return output_path


def get_drones_with_dynamic_geofences_flight_number(drone_file):
    with open(drone_file, newline='') as csv_file:
        reader = csv.reader(csv_file, delimiter=',', quotechar='|')
        drones_with_dynamic_fences = []
        for line in reader:
            if len(line) < 10:
                return []
            if line[10] != '':
                flight_number = line[1]
                duration = line[7]
                rectangle = line[8:8+4+1]
                drones_with_dynamic_fences.append([flight_number, duration, rectangle])
    return drones_with_dynamic_fences


def find_nodes_in_geofence(geofence, graph):
    x1,x2,y1,y2 = geofence[1:5]
    node_list = []
    for node in graph.nodes:
        if x1 <= graph.nodes[node]["x"] <= x2 and y1 <= graph.nodes[node]["y"] <= y2:
            node_list.append(node)
    return node_list


def generate_time_stamps(model, graph):
    with open("PLNE OUTPUT/drones_path_time_stamps", 'w+') as f:
        for drone in model.droneList:
            f.write(drone.flight_number)
            f.write("\n")
            for i, time_stamp in enumerate(drone.path_object.path_dict):
                f.write(str(i))
                f.write(" : ")
                time_str = str(int((time_stamp // 3600) // 10)) + str(int((time_stamp // 3600) % 10)) + ":"
                time_str += str(int((time_stamp % 3600) // 60 // 10)) + str(int((time_stamp % 3600) // 60 % 10)) + ":"
                time_str += str(int((time_stamp % 60) // 10)) + str(int((time_stamp % 10)))
                f.write(time_str)
                f.write(" : ")
                f.write(str(graph.nodes[drone.path_object.path_dict[time_stamp]]["x"]))
                f.write(" , ")
                f.write(str(graph.nodes[drone.path_object.path_dict[time_stamp]]["y"]))
                f.write("\n")
                # f.write(str(drone.path_object.path_dict[time_stamp]))
            f.write("\n")


def create_param(model, trajectories,trajectories_to_duration,horizontal_shared_nodes_list,climb_horiz_list,descent_horiz_list,climb_climb_list,descent_descent_list, fixed_flights_dict, fixed_flight_levels):
    # maxDelay = delay_max
    nbPt = [len(horizontal_shared_nodes_list), len(climb_horiz_list), len(descent_horiz_list), len(climb_climb_list),
            len(descent_descent_list)]
    all_pt = horizontal_shared_nodes_list + climb_horiz_list + descent_horiz_list + climb_climb_list + descent_descent_list

    k1 = [pt[0] for pt in all_pt]
    k2 = [pt[1] for pt in all_pt]
    t1 = [pt[2] for pt in all_pt]
    t2 = [pt[3] for pt in all_pt]
    sep12 = [pt[4] + TIME_MARGIN for pt in all_pt]
    sep21 = [pt[5] + TIME_MARGIN for pt in all_pt]
    A = trajectories.keys()
    K = []
    for a in trajectories:
        for k in trajectories[a]:
            K.append([k, trajectories_to_duration[k], a])
    fixed_intentions = []
    for fn in fixed_flights_dict:
        fixed_intentions.append(fixed_flights_dict[fn])
    param = Param.Param(model, A, K, nbPt, k1, k2, t1, t2, sep12, sep21, fixed_intentions, fixedLevels=fixed_flight_levels)

    # Print output :
    list_all = [horizontal_shared_nodes_list, climb_horiz_list, descent_horiz_list, climb_climb_list, descent_descent_list]
    tag_dict = {0:"hh", 1:"ch", 2:"dh", 3:"cc", 4:"dd"}
    tag_count = 0
    with open("PLNE OUTPUT/intersection_points", 'w') as file:
        for lines in list_all:
            for line in lines:
                to_write = tag_dict[tag_count] + " "
                for _pt in line:
                    to_write += str(_pt) + " "
                to_write += "\n"
                file.write(to_write)
            tag_count += 1

    return param


def create_fixed_param(problem, model, trajectories, trajectories_to_path, trajectories_to_fn, fn_order, current_time):
    Afix, Kfix, yfix, delayfix = [], [], [], []
    nb_trajs = sum([len(trajectories[drone_fn]) for drone_fn in trajectories.keys()])
    mon_vol = [fn_order.index(trajectories_to_fn[i]) for i in range(nb_trajs)]
    t = [None] * len(problem.param.A)
    for k in problem.param.K:
        if problem.x[k].x == 1:
            t[problem.param.mon_vol[k]] = k
    for chosen_traj in t:
        corresponding_flight = mon_vol[chosen_traj]
        drone = model.droneList[corresponding_flight]
        nb_of_traj_before = 0
        for _d in model.droneList[:corresponding_flight]:
            nb_of_traj_before += len(trajectories[_d.flight_number])
        path_object = trajectories_to_path[chosen_traj]
        arrival_time = max(path_object.path_dict.keys())
        if arrival_time > current_time:
            Afix.append(corresponding_flight)
            Kfix.append(chosen_traj)
            yfix.append(problem.y[corresponding_flight].x)
            delayfix.append(problem.delay[corresponding_flight].x)
    return Afix, Kfix, yfix, delayfix


def generate_SCN_v2(model, problem, trajectories, trajectories_to_fn, output_file):

    # Drone.VerticalIntegrator.integrate(problem.y[a].x * model.FL_sep).end_time()
    print(" GENERATING TRAJS :")
    graph = model.graph
    drone_delay_dict = dict()
    with open(output_file, 'w') as file:
        to_write = "00:00:00>CDMETHOD STATEBASED\n00:00:00>DTLOOK 20\n00:00:00>HOLD\n00:00:00>PAN 48.223775 16.337976\n00:00:00>ZOOM 50\n"
        file.write(to_write)
        # For each drone, find the chosen trajectory, FL, delay
        to_write_dict = dict()
        for k in problem.param.K:
            # If trajectory is selected
            if problem.x[k].x == 1:
                # Set the corresponding path to the drone
                drone = generate_trajectories.return_drone_from_flight_number(model, trajectories_to_fn[k])
                to_write_dict[drone] = []
                drone.path_object = trajectories[drone.flight_number][k][0]
                for a in problem.param.A:
                    if a == drone.flight_number:
                        fl_m = problem.y[a].x * model.FL_sep
                        fl_feet = fl_m * m_to_feet
                        delay = problem.delay[a].x * Param.delayStep
                        drone_delay_dict[drone] = delay
                        departure_time = drone.dep_time + delay
                dep_vertiport = " " + str(drone.departure_vertiport[1]) + " " + str(drone.departure_vertiport[0])
                first_node_x, first_node_y = graph.nodes[drone.path_object.path[0]]["x"], graph.nodes[drone.path_object.path[0]]["y"]
                qdr = BlueskySCNTools.qdrdist(drone.departure_vertiport[1], drone.departure_vertiport[0], first_node_y, first_node_x, 'qdr')
                # Write SCN
                # print("Drone FN : ", drone.flight_number, " dep :", departure_time)
                h, m, s = departure_time // 3600, (departure_time % 3600) // 60, departure_time % 60
                time_str = str(int(h // 10)) + str(int(h % 10)) + ":" + str(int(m // 10)) + str(int(m % 10)) + ":" + str(int(s // 10)) + str(int(s % 10))
                to_write = time_str + ">CRE " + drone.flight_number + " " + drone.drone_type + " " + dep_vertiport + " " + str(qdr) + " 0 " + str(0) + "\n"
                to_write_dict[drone].append(to_write)
                # file.write(to_write)
                to_write = time_str + ">ADDWAYPOINTS " + drone.flight_number + " " + dep_vertiport + "," + str(fl_feet) + ",,FLYBY,0,"
                # print(len(drone.path_object.path_dict.keys()), len(drone.path_object.turns))
                wpt = 0
                for wpt_time, node in drone.path_object.path_dict.items():
                    x, y = graph.nodes[node]["x"], graph.nodes[node]["y"]
                    if Drone.return_speed_from_angle(drone.path_object.turns[wpt], drone) != drone.speeds_dict["cruise"]:
                        to_write += str(y) + "," + str(x) + "," + str(fl_feet) + ",,TURNSPD," + str(Drone.return_speed_from_angle(drone.path_object.turns[wpt], drone) * ms_to_knots)
                    else:
                        to_write += str(y) + "," + str(x) + "," + str(fl_feet) + ",,FLYBY,0"
                    wpt += 1
                    if node != drone.path_object.path[-1]:
                        to_write += ","
                to_write += "\n"
                to_write_dict[drone].append(to_write)
                # file.write(to_write)
                to_write = time_str + ">" + drone.flight_number + " ATDIST " + str(y) + " " + str(x) + " " + str(drone_delete_dist)+ " DEL " + drone.flight_number + "\n"
                to_write_dict[drone].append(to_write)
                to_write = time_str + ">ALT " + drone.flight_number + " " + str(fl_feet) + "\n"
                to_write_dict[drone].append(to_write)
                to_write = time_str + ">ATALT " + drone.flight_number + " " + str(fl_feet) + " SPD " + drone.flight_number + " " + str(drone.speeds_dict["cruise"] * ms_to_knots) + "\n"
                to_write_dict[drone].append(to_write)
                to_write = time_str + ">ATALT " + drone.flight_number + " " + str(fl_feet) + " LNAV " + drone.flight_number + " ON\n"
                to_write_dict[drone].append(to_write)
                to_write = time_str + ">ATALT " + drone.flight_number + " " + str(fl_feet) + " VNAV " + drone.flight_number + " ON\n"
                to_write_dict[drone].append(to_write)

                wpt = 2
                for wpt_time, node in drone.path_object.path_dict.items():
                    to_write = time_str + ">RTA " + drone.flight_number + " " + drone.flight_number
                    wpt_str = str(wpt // 100) + str((wpt % 100) // 10) + str(wpt % 10)
                    to_write += wpt_str + " "
                    # rta_deviation = Drone.return_vertical_accel_time(0, Drone.vertical_speed) + (fl - Drone.return_vertical_accel_dist(0, Drone.vertical_speed)) / Drone.vertical_speed
                    # rta_deviation = fl / Drone.vertical_speed
                    rta_deviation = Drone.VerticalIntegrator.integrate(fl_m).end_time()
                    #
                    # print(rta_deviation)
                    if drone.path_object.turns[wpt - 2] > Drone.angle_intervals[0]:
                        rta_deviation -= drone.return_delay_from_angle(drone.path_object.turns[wpt - 2])
                    h, m, s = (wpt_time + delay + rta_deviation) // 3600, ((wpt_time + delay + rta_deviation) % 3600) // 60, (wpt_time + delay + rta_deviation) % 60
                    rta_time_str = str(int(h // 10)) + str(int(h % 10)) + ":" + str(int(m // 10)) + str(int(m % 10)) + ":" + str(int(s // 10)) + str(int(s % 10))
                    to_write += rta_time_str + "\n"
                    to_write_dict[drone].append(to_write)
                    # file.write(to_write)
                    wpt += 1

        # Sort the drones by actual dep time (wt delay) and write the SCN
        def return_drone_dep(_drone):
            return _drone.dep_time + drone_delay_dict[_drone]
        to_sort = []
        for drone in to_write_dict:
            to_sort.append(drone)
        sorted_drone_list = sorted(to_sort, key=return_drone_dep)
        for drone in sorted_drone_list:
            for to_write in to_write_dict[drone]:
                file.write(to_write)


solve_with_time_segmentation()

