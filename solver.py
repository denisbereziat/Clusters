import generate_trajectories
import Model as md
import osmnx
# import Param
# import PLNE
import Param2
import PLNE2
import tools
import BlueskySCNTools
import networkx
import csv

graph_file_path = "graph_files/total_graph_200m.graphml"
dual_graph_path = "graph_files/dual_total_graph_200m_with_geofences.graphml"
save_path = "graph_files/dual_total_graph_200m_with_geofences.graphml"
drone_list_file_path = 'graph_files/Intentions/1000drones.csv'
output_scenario = "PLNE OUTPUT/scenario.scn"
output_scenario_with_RTA = "PLNE OUTPUT/scenario_with_RTA.scn"
protection_area = 200
nb_FL = 10
delay_max = 120
FL_sep = 9.14  # in m
FL_min = 25
temps_sep_vertiport = 5
T_MAX_OPTIM = 120


def solve_with_time_segmentation():
    """"SOLVE the problem by using time segmentation.
    Dynamic geofence flights are sorted to be computed first and are always included in resolution"""
    print("-- INIT")
    print("Initialising graph")
    generation_params = protection_area, nb_FL, delay_max, FL_sep, FL_min, temps_sep_vertiport
    graph, graph_dual = md.init_graphs(graph_file_path, dual_path=dual_graph_path)
    if dual_graph_path is None:
        print("Saving dual graph")
        networkx.write_graphml(graph_dual, save_path)
    raw_graph = osmnx.load_graphml(graph_file_path)
    resolution = 100
    graph_hash = tools.hash_graph(graph, resolution)
    print("Graphs initialised")
    # Extract all deposit times
    deposit_time_list = tools.extract_deposit_times(drone_list_file_path)
    # No specified time to load all drones at first
    model = md.init_model(graph, graph_dual, drone_list_file_path, protection_area, graph_hash)
    # Dict used to store the drones and give them to the model.
    fn_to_drones_dict = dict()
    # drone_order = []
    for drone in model.droneList:
        fn_to_drones_dict[drone.flight_number] = drone
        # drone_order.append(drone.flight_number)
    model.total_drone_dict = fn_to_drones_dict
    # model.drone_order = drone_order
    drones_with_dynamic_fences = get_drones_with_dynamic_geofences_flight_number(drone_list_file_path)
    model.generation_params = generation_params
    print("NB DRONES :", len(model.droneList))
    print("-- INITIALISED")

    fixed_flights_dict = dict()

    print("Generating trajectories for dynamic geofences")
    # Keep only the dynamic geofences drones
    model.droneList = []
    for drone_with_fence in drones_with_dynamic_fences:
        drone_fn = drone_with_fence[0]
        model.droneList.append(fn_to_drones_dict[drone_fn])
    current_param = None
    # SOLVE
    traj_output, intersection_outputs, problem, param = solve_current_model(model, graph, raw_graph, graph_dual,
                                                                            current_param, fixed_flights_dict)
    trajectories, trajectories_to_fn, trajectories_to_duration, trajectories_to_path, fn_order = traj_output
    # Add the dynamic geofence drones to the fixed flights list
    # For each of the drone find the chosen traj, fl, delay
    for k in problem.param.K:
        if problem.x[k].x == 1:
            drone_fn = trajectories_to_fn[k]
            a = drone_fn
            # a = model.drone_order.index(drone_fn)
            fixed_flights_dict[drone_fn] = [a, k, problem.y[a].x, problem.delay[a].x]

    print("Starting resolution")
    # Load a ever increasing list of drone in the model, and don't forget the dynamic geofences ones
    sim_step = 25
    sim_size = 50
    sim_time = 0
    last_departure = max([model.total_drone_dict[drone_fn].dep_time for drone_fn in model.total_drone_dict])
    print("Last departure : ", last_departure)
    while sim_time < last_departure:
        sim_time += sim_step
        set_model_drone_list(model, sim_time + sim_size)
        print("\n\n--NB DRONES :", len(model.droneList))
        traj_output, intersection_outputs, problem, param = solve_current_model(model, graph, raw_graph, graph_dual,
                                                                                traj_output, fixed_flights_dict)
        trajectories, trajectories_to_fn, trajectories_to_duration, trajectories_to_path, fn_order = traj_output
        for k in problem.param.K:
            if problem.x[k].x == 1:
                drone_fn = trajectories_to_fn[k]
                # print(drone_fn)
                drone = model.total_drone_dict[drone_fn]
                if drone.dep_time < sim_time and drone_fn not in fixed_flights_dict:
                    a = drone_fn
                    fixed_flights_dict[drone_fn] = [a, k, problem.y[a].x, problem.delay[a].x]
        generate_scn(problem, fn_order, trajectories, model, trajectories_to_fn)

    # Generate SCN
    generate_scn(problem, fn_order, trajectories, model, trajectories_to_fn)


def solve_current_model(model, graph, raw_graph, graph_dual, current_param, fixed_flights_dict):
    ####
    # Solve current state of the model :
    # Generate trajectories
    print("-- Generating trajectories")
    trajectories, trajectories_to_fn, trajectories_to_duration, trajectories_to_path, fn_order = generate_trajectories.generate_trajectories(model, graph, raw_graph, graph_dual, current_param)
    trajectories_output = trajectories, trajectories_to_fn, trajectories_to_duration, trajectories_to_path, fn_order
    # Generate intersection points
    print("-- Intersection points")
    horizontal_shared_nodes_list,climb_horiz_list,descent_horiz_list,climb_climb_list,descent_descent_list = generate_trajectories.generate_intersection_points(trajectories, trajectories_to_fn, model, graph, graph_dual, raw_graph)
    intersection_outputs = horizontal_shared_nodes_list,climb_horiz_list,descent_horiz_list,climb_climb_list,descent_descent_list
    Afix, Kfix, yfix, delayfix = [], [], [], []
    for drone in fixed_flights_dict:
        Afix.append(fixed_flights_dict[drone][0])
        Kfix.append(fixed_flights_dict[drone][1])
        yfix.append(fixed_flights_dict[drone][2])
        delayfix.append(fixed_flights_dict[drone][3])
    # print("FIXED FLIGHTS :", Afix, Kfix, yfix, delayfix)
    # Create param
    param = create_param(trajectories,trajectories_to_duration,trajectories_to_fn,fn_order,Afix, Kfix,yfix, delayfix,horizontal_shared_nodes_list,climb_horiz_list,descent_horiz_list,climb_climb_list,descent_descent_list)
    # Create Problem
    problem = PLNE2.ProblemGlobal(param)
    problem.model.setParam("TimeLimit", T_MAX_OPTIM)
    # Solve
    problem.solve()
    # problem.printSolution()
    return trajectories_output, intersection_outputs, problem, param

#
# def solve():
#     ####
#     # INIT
#     trajectories = None
#     generation_params = protection_area, nb_FL, delay_max, FL_sep, FL_min, temps_sep_vertiport
#     # Init model and graphs
#     print("-- INIT")
#     print("Initialising graph")
#     graph, graph_dual = md.init_graphs(graph_file_path, dual_path=dual_graph_path)
#     print("Graphs initialised")
#     if dual_graph_path is None:
#         print("Saving dual graph")
#         networkx.write_graphml(graph_dual, save_path)
#     raw_graph = osmnx.load_graphml(graph_file_path)
#     print("Hash graph")
#     resolution = 100
#     graph_hash = tools.hash_graph(graph, resolution)
#     print("Initialising model")
#     # Extract all deposit times
#     deposit_time_list = tools.extract_deposit_times(drone_list_file_path)
#     model = md.init_model(graph, graph_dual, drone_list_file_path, protection_area, graph_hash, current_sim_time=deposit_time_list[0])
#     drones_with_dynamic_fences = get_drones_with_dynamic_geofences_flight_number(drone_list_file_path)
#     model.generation_params = generation_params
#     print("NB DRONES :", len(model.droneList))
#     current_param, current_time = None, deposit_time_list[0]
#
#     ####
#     # Generate trajectories
#     print("-- Generating trajectories")
#     trajectories, trajectories_to_fn, trajectories_to_duration, trajectories_to_path, fn_order = generate_trajectories.generate_trajectories(model, graph, raw_graph, graph_dual, current_param, current_time)
#     # Generate intersection points
#     print("Intersection points")
#     horizontal_shared_nodes_list,climb_horiz_list,descent_horiz_list,climb_climb_list,descent_descent_list = generate_trajectories.generate_intersection_points(trajectories, trajectories_to_fn, model, graph, graph_dual, raw_graph)
#     Afix, Kfix, yfix,delayfix = [], [], [], []
#     # Create param
#     param = create_param(trajectories,trajectories_to_duration,trajectories_to_fn,fn_order,Afix, Kfix,yfix, delayfix,horizontal_shared_nodes_list,climb_horiz_list,descent_horiz_list,climb_climb_list,descent_descent_list)
#
#     ####
#     # OUTPUT.DAT
#     print("Generating Output.dat")
#     generate_output(trajectories, horizontal_shared_nodes_list, climb_horiz_list, descent_horiz_list, climb_climb_list,
#                     descent_descent_list, model)
#
#     ####
#     # Create Problem
#     problem = PLNE.Problem(param)
#     problem.model.setParam("TimeLimit", T_MAX_OPTIM)
#     # Solve
#     problem.solve()
#     problem.printSolution()
#
#     ####
#     # Solve everything
#     for deposit_time_index in range(1, len(deposit_time_list)):
#         current_time = deposit_time_list[deposit_time_index]
#         print("\n\n-----CURRENT SIM TIME : ", current_time)
#
#         # Create fixed parameters
#         Afix, Kfix, yfix, delayfix = create_fixed_param(problem, model, trajectories, trajectories_to_path, trajectories_to_fn, fn_order, current_time)
#         # Add drones
#         model.add_drones_from_csv_file(drone_list_file_path, time=current_time)
#         print("NB DRONES :", len(model.droneList))
#
#         # generate new trajectories and add them to the list
#         current_param = trajectories, trajectories_to_fn, trajectories_to_duration, trajectories_to_path, fn_order
#
#         print("Generating trajectories")
#         trajectories, trajectories_to_fn, trajectories_to_duration, trajectories_to_path, fn_order = generate_trajectories.generate_trajectories(model, graph, raw_graph, graph_dual, current_param, current_time)
#         # Generate intersection points
#         print("Intersection points")
#         horizontal_shared_nodes_list, climb_horiz_list, descent_horiz_list, climb_climb_list, descent_descent_list = generate_trajectories.generate_intersection_points(trajectories, trajectories_to_fn, model, graph, graph_dual, raw_graph)
#         param = create_param(trajectories,trajectories_to_duration,trajectories_to_fn,fn_order,Afix, Kfix,yfix, delayfix,horizontal_shared_nodes_list,climb_horiz_list,descent_horiz_list,climb_climb_list,descent_descent_list)
#         # Create Problem
#         problem = PLNE.Problem(param)
#
#         # Solve
#         problem.solve()
#         # problem.printSolution()
#
#     # generate SCN
#     generate_scn(problem, fn_order, trajectories, model, trajectories_to_fn)
#
#     # Time stamps
#     generate_time_stamps(model, graph)
#     # RTA
#     generate_rta(model)


def set_model_drone_list(model, sim_time):
    """Add the drones from the total_list in the droneList depending on specified time
    Drones which are dynamic geofences drones (and should be the first of the list are added too"""
    model.droneList = []
    for drone_fn in model.total_drone_dict:
        drone = model.total_drone_dict[drone_fn]
        if drone.is_loitering_mission:
            model.droneList.append(drone)
        elif drone.dep_time <= sim_time:
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


def generate_scn(problem, fn_order, trajectories, model, trajectories_to_fn):
    ####
    # Generate SCN from the problem output
    # selected_traj = [None] * len(problem.param.A)  # All the selected trajectories after solving
    # selected_delay = [None] * len(problem.param.A)  # Match traj with the selected delay
    selected_delay = dict()
    # selected_fl = [None] * len(problem.param.A)  # Selected flight level
    selected_fl = dict()    # Get every selected trajectory for each drone :
    for k in problem.param.K:
        # If trajectory is selected
        if problem.x[k].x == 1:
            # selected_traj[problem.param.mon_vol[k]] = k
            # Set the corresponding path to the drone
            drone = generate_trajectories.return_drone_from_flight_number(model, trajectories_to_fn[k])
            drone.path_object = trajectories[drone.flight_number][k][0]
    alts = dict()
    for a in problem.param.A:
        selected_fl[a] = problem.y[a].x * FL_sep * 3.28084 + FL_min
        selected_delay[a] = problem.delay[a].x
        drone_fn = a
        drone = generate_trajectories.return_drone_from_flight_number(model, drone_fn)
        drone.dep_time = drone.dep_time + selected_delay[a]
        alts[drone_fn] = [selected_fl[a]] * len(drone.path_object.path)

    ####
    # BLUESKY
    print("Generating Bluesky SCN")
    scenario_dict = md.generate_scenarios(model, alts=alts)
    bst = BlueskySCNTools.BlueskySCNTools()
    bst.Dict2Scn(output_scenario, scenario_dict)
    # Add a line to enable ASAS in Bluesky
    with open(output_scenario, 'r+') as f:
        content = f.read()
        f.seek(0, 0)
        f.write("00:00:00>CDMETHOD STATEBASED")
        f.write("\n00:00:00>DTLOOK 20")
        f.write("\n")
        f.write(content)


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


def generate_rta(model):
    print("Creating file with RTA")
    with open(output_scenario, 'r') as scenario_without_RTA:
        with open(output_scenario_with_RTA, "w") as scenario_with_RTA:
            lines = scenario_without_RTA.readlines()
            current_drone_fn = None
            turning = False
            current_wpt = 0
            for line in lines:
                content = line.split(" ")
                if "CRE" in content[0]:
                    if current_drone_fn != content[1]:
                        current_drone_fn = content[1]
                        current_drone = generate_trajectories.return_drone_from_flight_number(model, current_drone_fn)
                        current_wpt = 1
                        turning = False
                        scenario_with_RTA.write(line)
                elif "ADDWPT" in content[0] and "FLYTURN" in content[2]:
                    turning = True
                    scenario_with_RTA.write(line)
                elif "ADDWPT" in content[0] and "TURNSPEED" in content[2]:
                    scenario_with_RTA.write(line)
                elif "ADDWPT" in content[0] and "FLYBY" in content[2]:
                    turning = False
                    scenario_with_RTA.write(line)
                elif "ADDWPT" in content[0]:
                    if turning:
                        scenario_with_RTA.write(line)
                    else:
                        to_write = content[0] + " " + content[1] + " " + content[2] + " " + content[3] + " " + content[4] + "\n"
                        scenario_with_RTA.write(to_write)
                        to_write = content[0][:-6]+"RTA" + " " + content[1] + " " + content[1]
                        wpt_nb = str(int(current_wpt // 100)) + str(int((current_wpt % 100) // 10)) + str(int(current_wpt % 10))
                        to_write += wpt_nb
                        rta_time = sorted(current_drone.path_object.path_dict.keys())[current_wpt]
                        rta_time_str = str(int((rta_time // 3600) // 10)) + str(int((rta_time // 3600) % 10)) + ":"
                        rta_time_str += str(int((rta_time % 3600) // 60 // 10)) + str(int((rta_time % 3600) // 60 % 10)) + ":"
                        rta_time_str += str(int((rta_time % 60) // 10)) + str(int((rta_time % 10)))
                        to_write += " " + rta_time_str + "\n"
                        scenario_with_RTA.write(to_write)
                    current_wpt += 1
                else:
                    scenario_with_RTA.write(line)


def generate_output(trajectories, horizontal_shared_nodes_list,climb_horiz_list, descent_horiz_list, climb_climb_list,descent_descent_list, model):
    with open("./PLNE OUTPUT/output.dat", 'w') as file:
        file.write("param nbflights := " + str(len(trajectories)) + ";\n")
        file.write("\nparam nbFL := " + str(nb_FL) + ";\n")
        file.write("param maxDelay := " + str(delay_max) + ";\n")
        file.write("param nbTrajs := " + str(sum([len(trajectories[d]) for d in trajectories])) + ";\n")
        file.write("param nbPtHor := " + str(len(horizontal_shared_nodes_list)) + ";\n")
        file.write("param nbPtClmb := " + str(len(climb_horiz_list)) + ";\n")
        file.write("param nbPtDesc := " + str(len(descent_horiz_list)) + ";\n")
        file.write("param nbPtDep := " + str(len(climb_climb_list)) + ";\n")
        file.write("param nbPtArr := " + str(len(descent_descent_list)) + ";\n")
        file.write("\n")
        file.write("param: d mon_vol :=")
        for drone in trajectories:
            for traj_id in trajectories[drone]:
                idx = generate_trajectories.get_drone_index_in_model_list(drone, model)
                file.write(
                    "\n" + str(traj_id + 1) + " " + str(int(trajectories[drone][traj_id][1])) + " " + str(idx + 1))
        file.write(";\n\n")
        file.write("param: k1 k2 t1 t2 sep12 sep21 :=")
        for index, traj in enumerate(horizontal_shared_nodes_list):
            file.write("\n")
            file.write(str(index + 1))
            file.write(" " + str(traj[0]) + " " + str(traj[1]))
            file.write(" " + str(round(traj[2], 1)) + " " + str(round(traj[3], 1)))
            file.write(" " + str(round(traj[4], 1)) + " " + str(round(traj[5], 1)))
        last_index = len(horizontal_shared_nodes_list)
        for index, traj in enumerate(climb_horiz_list):
            file.write("\n")
            file.write(str(last_index + index + 1))
            file.write(" " + str(traj[0]) + " " + str(traj[1]))
            file.write(" " + str(round(traj[2], 1)) + " " + str(round(traj[3], 1)))
            file.write(" " + str(round(traj[4], 1)) + " " + str(round(traj[5], 1)))
        last_index += len(climb_horiz_list)
        for index, traj in enumerate(descent_horiz_list):
            file.write("\n")
            file.write(str(last_index + index + 1))
            file.write(" " + str(traj[0]) + " " + str(traj[1]))
            file.write(" " + str(round(traj[2], 1)) + " " + str(round(traj[3], 1)))
            file.write(" " + str(round(traj[4], 1)) + " " + str(round(traj[5], 1)))
        last_index += len(descent_horiz_list)
        for index, traj in enumerate(climb_climb_list):
            file.write("\n")
            file.write(str(last_index + index + 1))
            file.write(" " + str(traj[0]) + " " + str(traj[1]))
            file.write(" " + str(round(traj[2], 1)) + " " + str(round(traj[3], 1)))
            file.write(" " + str(round(traj[4], 1)) + " " + str(round(traj[5], 1)))
        last_index += len(climb_climb_list)
        for index, traj in enumerate(descent_descent_list):
            file.write("\n")
            file.write(str(last_index + index + 1))
            file.write(" " + str(traj[0]) + " " + str(traj[1]))
            file.write(" " + str(round(traj[2], 1)) + " " + str(round(traj[3], 1)))
            file.write(" " + str(round(traj[4], 1)) + " " + str(round(traj[5], 1)))
        last_index += len(descent_descent_list)
        file.write(";\n")


def create_param(trajectories,trajectories_to_duration,trajectories_to_fn,fn_order,Afix, Kfix,yfix, delayfix,horizontal_shared_nodes_list,climb_horiz_list,descent_horiz_list,climb_climb_list,descent_descent_list):
    nb_flights = len(list(trajectories.keys()))
    nb_trajs = sum([len(trajectories[drone_fn]) for drone_fn in trajectories.keys()])
    # print("Devrait etre egal :", nb_trajs, len(trajectories_to_fn))
    maxDelay = delay_max
    nbPt = [len(horizontal_shared_nodes_list), len(climb_horiz_list), len(descent_horiz_list), len(climb_climb_list),
            len(descent_descent_list)]
    # d = [trajectories_to_duration[i] for i in range(nb_trajs)]  # Duration of horizontal_traj_i
    # mon_vol = [fn_order.index(trajectories_to_fn[i]) for i in range(nb_trajs)]
    # print("mon vol", mon_vol)
    all_pt = horizontal_shared_nodes_list + climb_horiz_list + descent_horiz_list + climb_climb_list + descent_descent_list
    k1 = [pt[0] for pt in all_pt]
    k2 = [pt[1] for pt in all_pt]
    t1 = [pt[2] for pt in all_pt]
    t2 = [pt[3] for pt in all_pt]
    sep12 = [pt[4] for pt in all_pt]
    sep21 = [pt[5] for pt in all_pt]
    # print(nb_flights, nb_trajs, maxDelay, nbPt, len(d), len(mon_vol), len(all_pt))
    A = trajectories.keys()
    K = []
    for a in trajectories:
        # k (id traj) d (duration) a (flight_id = flight number)
        for k in trajectories[a]:
            K.append([k, trajectories_to_duration[k], a])
    fixed_intentions = []
    fixed_levels = []
    param = Param2.Param(A, K, maxDelay, nbPt, k1, k2, t1, t2, sep12, sep21,
                         fixed_intentions, fixed_levels)
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


solve_with_time_segmentation()
