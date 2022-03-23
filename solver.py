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
import os

graph_file_path = "graph_files/total_graph_200m.graphml"
dual_graph_path = "graph_files/dual_total_graph_200m_with_geofences_no_demi_tour.graphml"
# dual_graph_path = None
save_path = "graph_files/dual_total_graph_200m_with_geofences.graphml"
output_directory = "PLNE OUTPUT"
input_directory = "graph_files/Intentions/M2_final_flight_intentions/flight_intentions"

HEURISTICS = 0.3
T_MAX_OPTIM_FL = 1800
T_MAX_OPTIM = 600 #not recommended as it may prevent finding a solution 
MIP_GAP = 1e-3
ms_to_knots = 1.94384
m_to_feet = 3.28084
drone_delete_dist = 0.0026997840172786176 * 3

# TODO a modifier ensuite
DYN_GEOFENCE_BUFFER = 0
TIME_MARGIN = 0
rta_time_deviation = 0


def main():
    input_files_list = [i for i in os.listdir(input_directory) if os.path.isfile(input_directory +"/"+ i)]
    for input_file in input_files_list:
        solve_with_time_segmentation(input_directory, output_directory, input_file)


def solve_with_time_segmentation(input_dir, output_dir, input_file_name):
    print("\n************************")
    print("SOLVING ", input_file_name)
    print("************************")
    """"SOLVE the problem by using time segmentation.
    Dynamic geofence flights are sorted to be computed first and are always included in resolution"""
    # Define input/output file paths
    input_file_path = input_dir + "/" + input_file_name
    output_file_path = output_dir + "/" + input_file_name[:-4] + ".scn"
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
    print("Loading flight intention file")
    # Load all drones at first
    model = md.init_model(graph, graph_dual, input_file_path, graph_hash)
    print("TOTAL NB DRONES :", len(model.total_drone_list))
    print("-- INITIALISED")

    #####
    # SET ALL FLIGHT LEVELS FOR AVAILABLE FLIGHTS AT T = 0
    set_model_drone_list(model, 0, math.inf)
    fixed_flight_levels_dict = solve_flight_levels_current_model(model, graph,raw_graph,graph_dual)

    fixed_flights_dict = {}
    model.drone_dict = {}
    #####
    # DYNAMIC GEOFENCES DRONES
    print("Generating trajectories for dynamic geofences")
    # Keep only the dynamic geofences drones
    if model.drones_with_dynamic_fences:
        model.drone_dict = dict(model.drones_with_dynamic_fences)
        # SOLVE
        print("\n-------------------------\n--NB DRONES :", len(model.drone_dict))
        traj_output, intersection_outputs, previous_solution = solve_current_model(model, graph, raw_graph, graph_dual, None, {}, {}, fixed_flight_levels_dict, {})
        trajectories, trajectories_to_fn, trajectories_to_duration, trajectories_to_path, fn_order = traj_output
        # Add the dynamic geofence drones to the fixed flights list
        for k in (k for (k, val) in previous_solution['x_val'].items() if val == 1):
            # For each of the drone find the chosen traj, fl, delay
            a = trajectories_to_fn[k]
            fixed_flights_dict[a] = [a, k, previous_solution['y_val'][a], previous_solution['delay_val'][a]]
            #calculate/set actual arr_time of this drone based on the chosen solution
            #hor travel time + ground delay + vertical movement time
            model.total_drone_dict[a].arr_time = trajectories_to_path[k].arr_time + previous_solution['delay_val'][a]*Param.delayStep + 2*Drone.VerticalIntegrator.integrate(model.FL_sep*previous_solution['y_val'][a]).end_time()
                
        # Create the geofenced nodes list
        geofence_time_intervals = dict()
        for drone_fn, drone  in model.drones_with_dynamic_fences.items():
            activation_time = drone.arr_time #we may even put some buffer 
            for node in find_nodes_in_geofence(drone.loitering_geofence, graph):
                interval = activation_time - DYN_GEOFENCE_BUFFER, activation_time + drone.loitering_geofence[0] + DYN_GEOFENCE_BUFFER
                if interval in geofence_time_intervals:
                    geofence_time_intervals[interval].append(node)
                else:
                    geofence_time_intervals[interval] = [node]
    else:
        print("\n-------------------------\n--No geofence drones")
        traj_output = None
        geofence_time_intervals = dict()
    
    #####
    # FULL RESOLUTION
    print("Starting resolution")
    # Load a ever increasing list of drone in the model, and don't forget the dynamic geofences ones
    sim_step = 300
    sim_size = 600
    sim_time = 0
    last_departure = max([drone.dep_time for drone in model.total_drone_dict.values()])
    print("Last departure : ", last_departure)
    previous_solution = {} #neglecting geofences that are anyway fixed
    while sim_time <= last_departure + sim_step:
        set_model_drone_list(model, sim_time,  sim_size)
        print("\n-------------------------\n--", len(fixed_flights_dict), " Total number of fixed flights till now (not all are considered)")
        print("-- WINDOW :", sim_time/60, "min - ", (sim_time+sim_size)/60, "min")
        print("-- NB DRONES :", len(model.drone_dict))
        traj_output, intersection_outputs, previous_solution = solve_current_model(model, graph, raw_graph, graph_dual, traj_output, fixed_flights_dict, geofence_time_intervals, fixed_flight_levels_dict, previous_solution)
        trajectories, trajectories_to_fn, trajectories_to_duration, trajectories_to_path, fn_order = traj_output
        for k in (k for (k, val) in previous_solution['x_val'].items() if val == 1):
            # For each of the drone find the chosen traj, fl, delay
            a = trajectories_to_fn[k]
            drone = model.total_drone_dict[a]
            if drone.dep_time < sim_time + sim_step and a not in fixed_flights_dict:
                fixed_flights_dict[a] = [a, k, previous_solution['y_val'][a], previous_solution['delay_val'][a]]
                #calculate/set actual arr_time of this drone based on the chosen solution
                #hor travel time + ground delay + vertical movement time
                drone.arr_time = trajectories_to_path[k].arr_time + previous_solution['delay_val'][a]*Param.delayStep + 2*Drone.VerticalIntegrator.integrate(model.FL_sep*previous_solution['y_val'][a]).end_time()
        sim_time += sim_step
    
    # Generate SCN
    generate_SCN_v2(model, fixed_flights_dict, trajectories, trajectories_to_fn, output_file_path)


def solve_current_model(model, graph, raw_graph, graph_dual, current_param, fixed_flights_dict, geofence_time_intervals, fixed_flight_levels_dict, previous_solution):
    ####
    # Solve current state of the model :
    # - Generate trajectories
    print("-- Generating trajectories")
    trajectories, trajectories_to_fn, trajectories_to_duration, trajectories_to_path, fn_order = generate_trajectories.generate_trajectories(model, graph, raw_graph, graph_dual, geofence_time_intervals, current_param)
    trajectories_output = trajectories, trajectories_to_fn, trajectories_to_duration, trajectories_to_path, fn_order
    # - Generate intersection points
    print("-- Intersection points")
    h_h_list,climb_h_list,descent_h_list,c_c_list,d_d_list = generate_trajectories.generate_intersection_points(trajectories, trajectories_to_fn, trajectories_to_path, model)
    intersection_outputs = h_h_list,climb_h_list,descent_h_list,c_c_list,d_d_list
    # - Create param
    param = create_param(model, trajectories, trajectories_to_duration, h_h_list, climb_h_list, descent_h_list, c_c_list, d_d_list, fixed_flights_dict, fixed_flight_levels_dict)
    # - Create Problem
    problem = PLNE.ProblemGlobal(param)
    if previous_solution: problem.setPartialSolution(previous_solution['x_val'], previous_solution['y_val'], previous_solution['delay_val'])
    problem.model.setParam("Heuristics", HEURISTICS)
    #problem.model.setParam("TimeLimit", T_MAX_OPTIM)
    problem.model.setParam("MIPGap", MIP_GAP)
    # Solve
    problem.solve()
    # problem.printSolution()
    #Set solution to initialise next problem
    x_val = {k: round(problem.x[k].x) for k in param.K}
    y_val = {a: round(problem.y[a].x) for a in param.A}
    delay_val = {a: round(problem.delay[a].x) for a in param.A}
    
    return trajectories_output, intersection_outputs, {'x_val': x_val, 'y_val': y_val, 'delay_val': delay_val}


def solve_flight_levels_current_model(model, graph, raw_graph, graph_dual):
    """Find a flight level assignation for all the drone of th model that reduce the number of conflicts"""
    # Generate only one trajectory for each drone (only the shortest)
    print("Generating shortest trajectories")
    trajectories, trajectories_to_fn, trajectories_to_duration, trajectories_to_path, fn_order = generate_trajectories.generate_trajectories(model, graph, raw_graph, graph_dual, [], None, number_of_trajectories=1)
    # Generate conflict matrix
    print("Computing interactions")
    interactions = generate_trajectories.generate_interaction(trajectories, model)
    A = model.drone_dict.keys() #although at this instance it is the same as total
    priorities = dict()
    for a, drone in model.drone_dict.items():
        priorities[a] = drone.priority
    param_level = Param.ParamLevelChoice(model, A, priorities, interactions)
    problem = PLNE.ProblemLevelChoice(param_level)
    problem.model.setParam("Heuristics", HEURISTICS)
    problem.model.setParam("TimeLimit", T_MAX_OPTIM_FL)
    problem.solve()
    #problem.printSolution()
    # Generate fixed_flight_levels_dict
    fixed_flight_levels = {}
    for a in problem.param.A:
        fixed_flight_levels[a] = (a, round(problem.y[a].x))
    return fixed_flight_levels


def set_model_drone_list(model, sim_time, sim_size):
    """Add the drones from the total_drone_list in the drone_dict depending on specified time
    Drones which are dynamic geofences drones (and should be the first of the list) are added too
    Fixed drones are removed once they couldn't influence solution anymore """
    model.drone_dict = {}
    for flight_number, drone in model.total_drone_dict.items():
        if drone.is_loitering_mission: #we let them always since they are not nombreux
            model.drone_dict[flight_number] = drone
        elif drone.dep_time <= sim_time + sim_size and not drone.arr_time < sim_time and not drone.deposit_time > sim_time:
            model.drone_dict[flight_number] = drone


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
    drones_with_dynamic_fences = []
    with open(drone_file, newline='') as csv_file:
        reader = csv.reader(csv_file, delimiter=',', quotechar='|')
        for line in reader:
            if len(line) < 10:
                return []
            if line[10] != '':
                flight_number = line[1]
                duration = line[7]
                rectangle = line[8:8+4+1]
                drones_with_dynamic_fences.append([flight_number, duration, rectangle])
    print("geo fence: ", drones_with_dynamic_fences)
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
        for flight_number, drone in model.drone_dict.items():
            f.write(flight_number)
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


def create_param(model, trajectories,trajectories_to_duration,horizontal_shared_nodes_list,climb_horiz_list,descent_horiz_list,climb_climb_list,descent_descent_list, fixed_flights_dict, fixed_flight_levels_dict):
    nbPt = [len(horizontal_shared_nodes_list), len(climb_horiz_list), len(descent_horiz_list), len(climb_climb_list),
            len(descent_descent_list)]
    all_pt = horizontal_shared_nodes_list + climb_horiz_list + descent_horiz_list + climb_climb_list + descent_descent_list

    k1 = [pt[0] for pt in all_pt]
    k2 = [pt[1] for pt in all_pt]
    t1 = [pt[2] for pt in all_pt]
    t2 = [pt[3] for pt in all_pt]
    sep12 = [pt[4] + TIME_MARGIN for pt in all_pt]
    sep21 = [pt[5] + TIME_MARGIN for pt in all_pt]
    A = model.drone_dict.keys()
    K = []
    priorities = {}
    fixed_intentions = []
    fixed_flight_levels_intentions = []
    for a, drone in model.drone_dict.items():
        priorities[a] = drone.priority
        for k in trajectories[a]:
            K.append([k, trajectories_to_duration[k], a])
        
        if a in fixed_flights_dict:
            fixed_intentions.append(fixed_flights_dict[a])
        elif a in fixed_flight_levels_dict:
            fixed_flight_levels_intentions.append(fixed_flight_levels_dict[a])
            
    print("--", len(fixed_intentions), "are fixed at this time.")

    param = Param.Param(model, A, priorities, K, nbPt, k1, k2, t1, t2, sep12, sep21, fixed_intentions, fixed_flight_levels_intentions)

    '''
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
    '''
    return param


def generate_SCN_v2(model, fixed_flights, trajectories, trajectories_to_fn, output_file):
    print(" GENERATING TRAJS :")
    graph = model.graph
    drone_delay_dict = dict()
    with open(output_file, 'w') as file:
        to_write = "00:00:00>CDMETHOD STATEBASED\n00:00:00>DTLOOK 20\n00:00:00>HOLD\n00:00:00>PAN 48.223775 16.337976\n00:00:00>ZOOM 50\n"
        file.write(to_write)
        # For each drone, find the chosen trajectory, FL, delay
        to_write_dict = dict()
        for index, content in fixed_flights.items():
            a, k, _fl, _delay = content
            delay = _delay * Param.delayStep
            fl_m  = _fl * model.FL_sep
        # for k in problem.param.K:
            # Set the corresponding path to the drone
            drone = generate_trajectories.return_drone_from_flight_number(model, trajectories_to_fn[k])
            to_write_dict[drone] = []
            drone.path_object = trajectories[drone.flight_number][k][0]
            fl_feet = fl_m * m_to_feet
            drone_delay_dict[drone] = delay
            departure_time = drone.dep_time + delay
            dep_vertiport = " " + str(drone.departure_vertiport[1]) + " " + str(drone.departure_vertiport[0])
            first_node_x, first_node_y = graph.nodes[drone.path_object.path[0]]["x"], graph.nodes[drone.path_object.path[0]]["y"]
            qdr = BlueskySCNTools.qdrdist(drone.departure_vertiport[1], drone.departure_vertiport[0], first_node_y, first_node_x, 'qdr')
            # Write SCN
            # print("Drone FN : ", drone.flight_number, " dep :", departure_time)
            h, m, s = departure_time // 3600, (departure_time % 3600) // 60, departure_time % 60
            time_str = str(int(h // 10)) + str(int(h % 10)) + ":" + str(int(m // 10)) + str(int(m % 10)) + ":" + str(int(s // 10)) + str(int(s % 10))
            if drone.is_loitering_mission:
                to_write = time_str + ">CRELOITER " + drone.flight_number + " " + drone.drone_type + " " + dep_vertiport + " " + str(
                    qdr) + " 0 " + str(0)
                fence = drone.loitering_geofence
                to_write += " " + str(fence[0])
                to_write += " " + str(fence[3]) + " " + str(fence[1]) + " " + str(fence[4]) + " " + str(fence[1])
                to_write += " " + str(fence[4]) + " " + str(fence[2]) + " " + str(fence[3]) + " " + str(fence[2])
                to_write += "\n"
            else:
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
            # Have the drone stop, descend and delete itself when it reaches the vertiport
            to_write = time_str + ">ALT " + drone.flight_number + " " + str(fl_feet) + "\n"
            to_write_dict[drone].append(to_write)
            to_write = time_str + ">ATALT " + drone.flight_number + " " + str(fl_feet) + " SPD " + drone.flight_number + " " + str(drone.speeds_dict["cruise"] * ms_to_knots) + "\n"
            to_write_dict[drone].append(to_write)
            to_write = time_str + ">ATALT " + drone.flight_number + " " + str(fl_feet) + " LNAV " + drone.flight_number + " ON\n"
            to_write_dict[drone].append(to_write)
            to_write = time_str + ">ATALT " + drone.flight_number + " " + str(fl_feet) + " VNAV " + drone.flight_number + " ON\n"
            to_write_dict[drone].append(to_write)
            if drone.is_loitering_mission:
                to_write = time_str + ">" + drone.flight_number + " ATDIST " + str(y) + " " + str(x) + " " + str(drone_delete_dist) + " DELLOITER " + drone.flight_number + "\n"
                to_write_dict[drone].append(to_write)
            else:
                to_write = time_str + ">" + drone.flight_number + " ATDIST " + str(y) + " " + str(x) + " " + str(drone_delete_dist) + " SPD " + drone.flight_number + "  0\n"
                to_write_dict[drone].append(to_write)
                to_write = time_str + ">" + drone.flight_number + " ATDIST " + str(y) + " " + str(x) + " " + str(drone_delete_dist) + " ALT " + drone.flight_number + "  0\n"
                to_write_dict[drone].append(to_write)
                to_write = time_str + ">" + drone.flight_number + " ATDIST " + str(y) + " " + str(x) + " " + str(drone_delete_dist) + " ATALT " + drone.flight_number + " 0.2 DEL " + drone.flight_number + " \n"
                to_write_dict[drone].append(to_write)

            wpt = 2
            for wpt_time, node in drone.path_object.path_dict.items():
                to_write = time_str + ">RTA " + drone.flight_number + " " + drone.flight_number
                wpt_str = str(wpt // 100) + str((wpt % 100) // 10) + str(wpt % 10)
                to_write += wpt_str + " "
                # rta_deviation = Drone.return_vertical_accel_time(0, Drone.vertical_speed) + (fl - Drone.return_vertical_accel_dist(0, Drone.vertical_speed)) / Drone.vertical_speed
                # rta_deviation = fl / Drone.vertical_speed
                rta_deviation = Drone.VerticalIntegrator.integrate(fl_m).end_time()
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


main()
