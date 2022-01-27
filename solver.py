import generate_trajectories
import Model as md
import osmnx
import Param
import PLNE
import tools
import BlueskySCNTools

graph_file_path = "graph_files/total_graph_200m.graphml"
dual_graph_path = "graph_files/dual_total_graph_200m.graphml"
drone_list_file_path = 'graph_files/Intentions/1000_flight_intention_diff_deposit_time.csv'
output_scenario = "PLNE OUTPUT/scenario.scn"
output_scenario_with_RTA = "PLNE OUTPUT/scenario_with_RTA.scn"
protection_area = 100
nb_FL = 10
delay_max = 120
FL_sep = 9.14
FL_min = 25
temps_sep_vertiport = 5
T_MAX_OPTIM = 3600


def solve():
    trajectories = None
    generation_params = protection_area, nb_FL, delay_max, FL_sep, FL_min, temps_sep_vertiport
    # Init model and graphs
    print("-- INIT")
    print("Initialising graph")
    graph, graph_dual = md.init_graphs(graph_file_path, dual_path=dual_graph_path)
    # networkx.write_graphml(graph_dual, dual_graph_path)
    raw_graph = osmnx.load_graphml(graph_file_path)
    print("Hash graph")
    resolution = 100
    graph_hash = tools.hash_graph(graph, resolution)

    print("Initialising model")
    # Extract all deposit times
    deposit_time_list = tools.extract_deposit_times(drone_list_file_path)
    # print(deposit_time_list)
    model = md.init_model(graph, graph_dual, drone_list_file_path, protection_area, graph_hash, current_sim_time=deposit_time_list[0])
    model.generation_params = generation_params
    # model.hash_map = graph_hash
    print("NB DRONES :", len(model.droneList))
    current_param, current_time = None, deposit_time_list[0]

    ####
    # Generate trajectories
    print("-- GENERATION")
    print("Generating trajectories")
    # TODO Encore quelques problemes avec certains drones particuliers ou on arriv pas a generer de traj alternatives dans l'exemple low traffic
    trajectories, trajectories_to_fn, trajectories_to_duration, trajectories_to_path, fn_order = generate_trajectories.generate_trajectories(model, graph, raw_graph, graph_dual, current_param, current_time)
    # Generate intersection points
    print("Intersection points")
    horizontal_shared_nodes_list,climb_horiz_list,descent_horiz_list,climb_climb_list,descent_descent_list = generate_trajectories.generate_intersection_points(trajectories, trajectories_to_fn, model, graph, graph_dual, raw_graph)
    Afix, Kfix, yfix,delayfix = [], [], [], []
    # Create param for problem
    # nb_flights = len(list(trajectories.keys()))
    # nb_trajs = sum([len(trajectories[drone_fn]) for drone_fn in trajectories.keys()])
    # # print("Devrait etre egal :", nb_trajs, len(trajectories_to_fn))
    # maxDelay = delay_max
    # nbPt = [len(horizontal_shared_nodes_list), len(climb_horiz_list), len(descent_horiz_list), len(climb_climb_list), len(descent_descent_list)]
    # d = [trajectories_to_duration[i] for i in range(nb_trajs)] # Duration of horizontal_traj_i
    # mon_vol = [fn_order.index(trajectories_to_fn[i]) for i in range(nb_trajs)]
    # print("mon vol",mon_vol)
    # all_pt = horizontal_shared_nodes_list + climb_horiz_list + descent_horiz_list + climb_climb_list + descent_descent_list
    # k1 = [pt[0] for pt in all_pt]
    # k2 = [pt[1] for pt in all_pt]
    # t1 = [pt[2] for pt in all_pt]
    # t2 = [pt[3] for pt in all_pt]
    # sep12 = [pt[4] for pt in all_pt]
    # sep21 = [pt[5] for pt in all_pt]
    # print(nb_flights, nb_trajs, maxDelay, nbPt, len(d), len(mon_vol), len(all_pt))
    # Afix = []
    # Kfix = []
    # yfix = []
    # delayfix = []
    # param = Param.Param(nb_flights, nb_trajs, maxDelay, nbPt, d, mon_vol, k1, k2, t1, t2, sep12, sep21, Afix, Kfix, yfix, delayfix)
    param = create_param(trajectories,trajectories_to_duration,trajectories_to_fn,fn_order,Afix, Kfix,yfix, delayfix,horizontal_shared_nodes_list,climb_horiz_list,descent_horiz_list,climb_climb_list,descent_descent_list)

    ####
    # OUTPUT.DAT
    print("Generating Output.dat")
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
                # print(drone_trajectories_dict[drone][traj_id])
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

    # Create Problem
    problem = PLNE.Problem(param)
    problem.model.setParam("TimeLimit", T_MAX_OPTIM)
    # Solve
    problem.solve()
    problem.printSolution()

    for deposit_time_index in range(1, len(deposit_time_list)):
        current_time = deposit_time_list[deposit_time_index]
        print("\n\n-----CURRENT SIM TIME : ", current_time)

        # Create fixed parameters
        Afix, Kfix, yfix, delayfix = create_fixed_param(problem, model, trajectories, trajectories_to_path, trajectories_to_fn, fn_order, current_time)
        # Add drones
        model.add_drones_from_csv_file(drone_list_file_path, time=current_time)
        print("NB DRONES :", len(model.droneList))

        # generate new trajectories and add them to the list
        current_param = trajectories, trajectories_to_fn, trajectories_to_duration, trajectories_to_path, fn_order

        print("Generating trajectories")
        trajectories, trajectories_to_fn, trajectories_to_duration, trajectories_to_path, fn_order = generate_trajectories.generate_trajectories(model, graph, raw_graph, graph_dual, current_param, current_time)
        # Generate intersection points
        print("Intersection points")
        horizontal_shared_nodes_list, climb_horiz_list, descent_horiz_list, climb_climb_list, descent_descent_list = generate_trajectories.generate_intersection_points(trajectories, trajectories_to_fn, model, graph, graph_dual, raw_graph)
        param = create_param(trajectories,trajectories_to_duration,trajectories_to_fn,fn_order,Afix, Kfix,yfix, delayfix,horizontal_shared_nodes_list,climb_horiz_list,descent_horiz_list,climb_climb_list,descent_descent_list)
        # Create Problem
        problem = PLNE.Problem(param)

        # Solve
        problem.solve()
        # problem.printSolution()

    # Generate SCN from the problem output
    selected_traj = [None] * len(problem.param.A)  # All the selected trajectories after solving
    selected_delay = [None] * len(problem.param.A)  # Match traj with the selected delay
    selected_fl = [None] * len(problem.param.A)  # Selected flight level
    # Get every selected trajectory for each drone :
    for k in problem.param.K:
        # If trajectory is selected
        if problem.x[k].x == 1:
            selected_traj[problem.param.mon_vol[k]] = k
            # Set the corresponding path to the drone
            drone = generate_trajectories.return_drone_from_flight_number(model, trajectories_to_fn[k])
            drone.path_object = trajectories[drone.flight_number][k][0]
    alts = dict()
    for a in problem.param.A:
        selected_fl[a] = problem.y[a].x * FL_sep + FL_min
        selected_delay[a] = problem.delay[a].x
        drone_fn = fn_order[a]
        drone = generate_trajectories.return_drone_from_flight_number(model, drone_fn)
        drone.dep_time = drone.dep_time + selected_delay[a]
        alts[drone_fn] = [selected_fl[a]] * len(drone.path_object.path)


    # print("Write FN trajectories match")
    # with open("./PLNE OUTPUT/FN_traj.dat", 'w+') as file:
    #     for drone in trajectories:
    #         for traj_id in trajectories[drone]:
    #             file.write(str(drone) + " " + str(traj_id) + " \n")
    #             for _s in trajectories[drone][traj_id][0]:
    #                 file.write(_s + " ")
    #             file.write("\n")

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




def create_param(trajectories,trajectories_to_duration,trajectories_to_fn,fn_order,Afix, Kfix,yfix, delayfix,horizontal_shared_nodes_list,climb_horiz_list,descent_horiz_list,climb_climb_list,descent_descent_list):
    nb_flights = len(list(trajectories.keys()))
    nb_trajs = sum([len(trajectories[drone_fn]) for drone_fn in trajectories.keys()])
    # print("Devrait etre egal :", nb_trajs, len(trajectories_to_fn))
    maxDelay = delay_max
    nbPt = [len(horizontal_shared_nodes_list), len(climb_horiz_list), len(descent_horiz_list), len(climb_climb_list),
            len(descent_descent_list)]
    d = [trajectories_to_duration[i] for i in range(nb_trajs)]  # Duration of horizontal_traj_i
    mon_vol = [fn_order.index(trajectories_to_fn[i]) for i in range(nb_trajs)]
    # print("mon vol", mon_vol)
    all_pt = horizontal_shared_nodes_list + climb_horiz_list + descent_horiz_list + climb_climb_list + descent_descent_list
    k1 = [pt[0] for pt in all_pt]
    k2 = [pt[1] for pt in all_pt]
    t1 = [pt[2] for pt in all_pt]
    t2 = [pt[3] for pt in all_pt]
    sep12 = [pt[4] for pt in all_pt]
    sep21 = [pt[5] for pt in all_pt]
    # print(nb_flights, nb_trajs, maxDelay, nbPt, len(d), len(mon_vol), len(all_pt))

    param = Param.Param(nb_flights, nb_trajs, maxDelay, nbPt, d, mon_vol, k1, k2, t1, t2, sep12, sep21, Afix, Kfix,
                        yfix, delayfix)
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


solve()
