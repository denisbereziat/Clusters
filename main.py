import networkx as nx
import Astar2 as a2
import Graph_repr as gr
import Model as md
import matplotlib.pyplot as plt
import Cluster as cl
import time
import dual_graph
import tools
import BlueskySCNTools
import Path

# PARAMETERS
graph_file_path = "graph_files/processed_graphM2.graphml"
drone_list_file_path = 'graph_files/drones.txt'
drone_with_deposit_time_list_file_path = 'graph_files/drones_with_deposit_times.txt'
scenario_path = r'M2_Test_Scenario_new.scn'
display_metrics = False
turn_enabled = True
turn_weight = 20
minimum_angle_to_apply_added_weight = 45
protection_area = 50
max_iteration = 100
time_interval_discretization = 5
constraints_nodes = {}
bool_draw_intermediary_solutions = False
bool_draw_final_solutions = True
# Limit how many permutations to test when solving the clusters
# TODO add a percent of best value as stop parameter too
max_number_of_permutations = 10


def solve_with_announce_time():
    start_time = time.time()

    # Extract the list of flight plan's deposit times (no duplicates)
    deposit_times_list = extract_deposit_times(drone_with_deposit_time_list_file_path)

    # Init a model that will be used to store all the data through the iterations
    final_model, _g, _g_dual = init_model(graph_file_path, drone_list_file_path, protection_area, "00:00:00")
    final_model.set_graph_dual(_g_dual)
    # Init the drones path
    for drone in final_model.droneList:
        drone.path_object = Path.Path(drone.dep_time, [])

    # Iterate over all the different flight plan's deposit time
    for sim_time_index, current_sim_time in enumerate(deposit_times_list):
        # Changing the time in seconds and determining the next time of the simulation
        print("\nCurrent_sim_time: ", current_sim_time)
        current_sim_time_s = float(current_sim_time[0:2])*3600 + float(current_sim_time[3:5])*60 + float(current_sim_time[6:8])
        if sim_time_index+1 < len(deposit_times_list):
            next_sim_time = deposit_times_list[sim_time_index+1]
            next_sim_time_s = float(next_sim_time[0:2])*3600 + float(next_sim_time[3:5])*60 + float(next_sim_time[6:8])

        # Initialise a dict that will be used to store the initial constraints for the model, these constraints are due
        # to the drones currently at the moment of the model initialisation
        model_initial_constraints_dict = dict()
        # Initialise the model and graph that will be used
        model, graph, graph_dual = init_model(graph_file_path, drone_with_deposit_time_list_file_path,
                                              protection_area, current_sim_time)
        model.set_graph_dual(graph_dual)

        # LOAD DRONES THAT WERE ALREADY FLYING
        # To load all the drones that have already taken-off, we check for each drone of the model used to save if it
        # has a path, and if he has, where he is at the moment of the current_sim_time (flying or landed)
        print("Loading drones")
        already_landed_drone_list = []
        # TODO don't redo all calculations of shortest path if there are no conflicts
        for drone in model.droneList:
            index_drone = [_d.flight_number for _d in final_model.droneList].index(drone.flight_number)
            final_drone = final_model.droneList[index_drone]
            node_times = list(final_drone.path_object.path_dict.keys())
            node_times.sort()
            # If there is at least one item in the path, it means that the drone was already given one
            # in the previous simulation time
            if len(node_times) >= 1:
                drone.dep_time = node_times[-1]
                drone.dep = final_drone.path_object.path_dict[drone.dep_time]
                # Check if the drone.arr = drone.dep and then the time of arrival
                if drone.dep == drone.arr:
                    already_landed_drone_list.append(drone)
                    # If the time of arrival is after the current sim_time, the drone hasn't landed yet so it needs
                    # to be added to the initial constraints for the model
                    if drone.dep_time > current_sim_time_s:
                        model_initial_constraints_dict[drone.dep_time] = [drone.dep, drone.dep_time, drone.dep, drone]
        for drone in already_landed_drone_list:
            model.droneList.remove(drone)
        print("Drone flying at this time : ", [_d.flight_number for _d in model.droneList])
        for drone in model.droneList:
            model_initial_constraints_dict[drone.dep_time] = [drone.dep, drone.dep_time, drone.dep, drone]
        # SOLVE
        # Solve the problem with the current parameters
        model = solve_clusters_with_dual_and_constraints(model)
        # SAVE DRONES
        # Save the solutions found up to just one node after the next simulation time
        save_drones(model, final_model, sim_time_index, deposit_times_list, next_sim_time_s)


    # DRAW SOLUTIONS
    if bool_draw_final_solutions:
        for drone in final_model.droneList:
            drone.path_object.discretize_path(time_interval_discretization, graph, drone)
        draw_solution(final_model)

    # Save paths in a file
    print("Saving all paths \n")
    file = open("path_with_turns.txt", "w")
    for drone in final_model.droneList:
        file.write(
            "\nDrone ID :" + str(final_model.droneList.index(drone)) + "\n" + str(drone.path_object.path) + "\n")
    file.close()

    # Generate Bluesky scenarios
    scenario_dict = generate_scenarios(final_model)
    bst = BlueskySCNTools.BlueskySCNTools()
    bst.Dict2Scn(scenario_path, scenario_dict)
    # Add a line to enable ASAS in Bluesky
    with open(scenario_path, 'r+') as f:
        content = f.read()
        f.seek(0, 0)
        f.write("00:00:00>CDMETHOD STATEBASED")
        f.write("\n00:00:00>DTLOOK 20")
        f.write("\n")
        f.write(content)

    # model, graph, graph_dual = init_model(graph_file_path, drone_list_file_path, protection_area, "00:00:00")
    # model.set_graph_dual(graph_dual)
    # solve_clusters_with_dual_and_constraints(model)
    print('Time taken by algorithm:', time.time() - start_time, 's')


def solve_clusters_with_dual_and_constraints(model):
    """Solve the pathing problem for the drones of the specified file on the graph using the cluster method and the A*
        algorithm on a dual graph of the given graph, which takes into account the influence of turns on drones speed"""

    # 1 Initialise the graph (normal and dual)
    graph = model.graph
    graph_dual = model.graph_dual

    # 2 Compute the shortest path for each drone using the A* algorithm on the dual graph
    print("Computing shortest path for each drone")
    model, initial_total_flight_time, initial_total_flight_distance = compute_all_shortest_paths(model, graph)
    # print("Total flight time:", initial_total_flight_time)
    # print("Total flight distance:", initial_total_flight_distance)

    # 3 Find conflicts
    conflicts = model.find_conflicts(graph)
    print('Initial number of conflicts: ', len(conflicts))

    # 4 Solve
    iteration_count = 0
    while len(conflicts) != 0 and iteration_count < max_iteration:
        iteration_count += 1
        gr.reset_color(graph)
        # Sorting conflicts by time to solve the earliest ones first
        conflicts.sort(key=lambda x: x[2])
        current_conflict = conflicts.pop(0)
        drone = model.droneList[current_conflict[0]]
        edge = drone.find_current_edge(current_conflict[2], graph)
        conflicting_drones = (model.droneList[current_conflict[0]], model.droneList[current_conflict[1]])
        cluster = cl.Cluster(edge[0], conflicting_drones, graph)
        # Find the drones passing through the cluster
        cluster.find_drones(model, current_conflict[2])
        # Solve the cluster
        cluster.solve_cluster_dual(model, max_number_of_permutations)
        # Redo the conflict search to take into account the modifications
        conflicts = model.find_conflicts(graph)
    # Display the conflicts left if there are any
    if len(conflicts) != 0:
        print('Conflicts lefts :', len(conflicts))
        print([[model.droneList[c[0]].flight_number, model.droneList[c[1]].flight_number, c[2]] for c in conflicts])
    return model


def extract_deposit_times(filepath):
    """Extract just the flight plans deposit times"""
    deposit_times_list = []
    with open(filepath) as f:
        for line in f.readlines():
            deposit_time = line[0:8]
            if deposit_time not in deposit_times_list:
                deposit_times_list.append(deposit_time)
    return deposit_times_list


def save_drones(model, final_model, sim_time_index, deposit_times_list, next_sim_time_s):
    for temp_drone in model.droneList:
        sorted_node_times = list(temp_drone.path_object.path_dict.keys())
        sorted_node_times.sort()
        for drone in final_model.droneList:
            if drone.flight_number == temp_drone.flight_number:
                for node_time in sorted_node_times:
                    if node_time not in drone.path_object.path_dict:
                        # If it isn't the last iteration, we want to add all points in between
                        # deposit_times plus the next one
                        if sim_time_index + 1 < len(deposit_times_list):
                            if node_time < next_sim_time_s:
                                drone.path_object.path_dict[node_time] = temp_drone.path_object.path_dict[node_time]
                                drone.path_object.path.append(drone.path_object.path_dict[node_time])
                            else:
                                drone.path_object.path_dict[node_time] = temp_drone.path_object.path_dict[node_time]
                                drone.path_object.path.append(drone.path_object.path_dict[node_time])
                                break
                        # If it is the last iteration we add all nodes to the end
                        else:
                            drone.path_object.path_dict[node_time] = temp_drone.path_object.path_dict[node_time]
                            drone.path_object.path.append(drone.path_object.path_dict[node_time])


def generate_scenarios(model):
    scenario_dict = dict()
    for drone in model.droneList:
        drone_id = drone.flight_number
        scenario_dict[drone.flight_number] = dict()
        lats, lons, turns = extract_lat_lon_turn_bool_from_path(drone, model)
        scenario_dict[drone_id]['start_time'] = min(drone.path_object.path_dict.keys())
        # Add lats
        scenario_dict[drone_id]['lats'] = lats
        # Add lons
        scenario_dict[drone_id]['lons'] = lons
        # Add turnbool
        scenario_dict[drone_id]['turnbool'] = turns
        # Add alts, everyone flies at 25 feet altitude for now
        scenario_dict[drone_id]['alts'] = [25] * len(lats)
    return scenario_dict


def draw_solution(model):
    print("Drawing solutions")
    graph = model.graph
    for drone in model.droneList:
        gr.draw_solution(graph, drone, show_id=False, show_discretized=False, show_time=True, show=False)
        plt.savefig("solutions/plt_sol_{}.png".format(drone.flight_number), dpi=400)
        plt.close()


def extract_lat_lon_turn_bool_from_path(drone, model):
    lats = []
    lon = []
    graph = model.graph
    for node in drone.path_object.path:
        node_x = graph.nodes[node]['x']
        node_y = graph.nodes[node]['y']
        lats.append(node_y)
        lon.append(node_x)
    turn_bool = [False for _i in range(len(lats))]
    for i in range(1, len(turn_bool)-1):
        turn_bool[i] = turn_bool_function([lon[i-1], lats[i-1]], [lon[i], lats[i]], [lon[i+1], lats[i+1]])
    return lats, lon, turn_bool


def init_model(graphml_path, drone_list_path, protection_area_size, current_sim_time):
    """ initialise a model instance with a primal graph and a dual one and load drones from the specified time taking
    into account the current_time so the drones announced after this time aren't loaded."""
    graph = nx.read_graphml(graphml_path)
    model = md.Model(graph, protection_area_size)
    # Creating the dual graph from the primal
    graph_dual = dual_graph.create_dual(model, turn_cost_function)
    model.droneList = []
    model.add_drones_from_file(drone_list_path, current_sim_time)
    return model, graph, graph_dual


def compute_all_shortest_paths(model, graph):
    """Compute the shortest path for each drone using the A* algorithm
    on the given model using the graph and the dual graph"""
    initial_total_flight_time = 0
    initial_total_flight_distance = 0
    drone_path_calculated = 0
    for drone in model.droneList:
        drone_path_calculated += 1
        # Changing the arrival and departures nodes to the Sources and Terminals node for the dual graph
        drone_dep_dual = ("S" + drone.dep, drone.dep)
        drone_arr_dual = (drone.arr, drone.arr + "T")
        # Performing A* algorithm on the dual graph, the returned path is given using the primal graph.
        shortest_path = a2.astar_dual(model, drone_dep_dual, drone_arr_dual, drone, drone.dep_time,
                                      primal_constraint_nodes_dict=constraints_nodes)
        # Adding the outputs to the drone object
        # Ici drone_path c'est un objet Path
        shortest_path.set_path(shortest_path.path, graph, model.graph_dual, drone)
        shortest_path.discretize_path(5, graph, drone)
        shortest_path.flight_time_and_distance(graph, drone)
        drone.path_object = shortest_path
        drone.time = shortest_path.flightTime
        initial_total_flight_time += shortest_path.flightTime
        initial_total_flight_distance += shortest_path.flightDistance
    return model, initial_total_flight_time, initial_total_flight_distance


def compute_and_display_results(model, start_time):
    """Compute store and display metrics on the performance of the final given paths for the model"""
    total_delay, no_delay_count, max_delay = 0, 0, 0
    total_flight_time, total_flight_distance = 0, 0
    max_added_flight_time, no_added_flight_time_count = 0, 0
    for drone in model.droneList:
        total_flight_time += drone.path_object.flightTime
        added_flight_time = drone.path_object.flightTime - drone.time
        if added_flight_time > max_added_flight_time:
            max_added_flight_time = added_flight_time
        if added_flight_time == 0:
            no_added_flight_time_count += 1
        total_flight_distance += drone.path_object.flightDistance
        delay_drone = drone.path_object.hStart - drone.dep_time
        for node in drone.path_object.delay:
            delay_drone += drone.path_object.delay[node]
        total_delay += delay_drone
        if delay_drone == 0:
            no_delay_count += 1
        if delay_drone > max_delay:
            max_delay = delay_drone
    print("Total flight time:", total_flight_time)
    print("Total flight distance:", total_flight_distance)
    print("Maximum overtime:", max_added_flight_time, "and no added:", no_added_flight_time_count)
    print('Total delay:', total_delay, 'including with no delay:', no_delay_count, 'and max delay:', max_delay)
    print('Time taken by algorithm:', time.time() - start_time, 's')
    metrics = {"total_delay": total_delay, "no_delay_count": no_delay_count, "total_flight_time": total_flight_time,
               "total_flight_distance": total_flight_distance, "max_added_flight_time": max_added_flight_time,
               "no_added_flight_time_count": no_added_flight_time_count}
    return metrics


def turn_cost_function(node1, node2, node3):
    """The function used to determine the cost to add to the edge in the dual graph to take into account the effect
    of turns on the drone speed and flight_time, it returns the 2 parts of the added cost of the turn (deceleration
    and acceleration) and the total added cost in time"""
    # TODO as of now we consider only one speed profile, we can either make a graph per drone, or
    #  just return the turn angle and do calculations each time
    if not turn_enabled:
        return 0, 0, 0
    x1, y1 = float(node1["x"]), float(node1["y"])
    x2, y2 = float(node2["x"]), float(node2["y"])
    x3, y3 = float(node3["x"]), float(node3["y"])
    v1 = (x2 - x1, y2 - y1)
    v2 = (x3 - x2, y3 - y2)
    angle = tools.angle_btw_vectors(v1, v2)
    if angle > minimum_angle_to_apply_added_weight:
        pre_turn_cost = (turn_weight * angle/180)/2
        post_turn_cost = (turn_weight * angle/180)/2
        total_turn_cost = pre_turn_cost + post_turn_cost
        return total_turn_cost, pre_turn_cost, post_turn_cost
    else:
        return 0, 0, 0


def turn_bool_function(node1, node2, node3):
    x1, y1 = float(node1[0]), float(node1[1])
    x2, y2 = float(node2[0]), float(node2[1])
    x3, y3 = float(node3[0]), float(node3[1])
    v1 = (x2 - x1, y2 - y1)
    v2 = (x3 - x2, y3 - y2)
    angle = tools.angle_btw_vectors(v1, v2)
    if angle > minimum_angle_to_apply_added_weight:
        return True
    else:
        return False


if __name__ == "__main__":
    solve_with_announce_time()
