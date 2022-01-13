import generate_trajectories
import Model as md
import osmnx
import Param
import PLNE

graph_file_path = "graph_files/total_graph_200m.graphml"
dual_graph_path = "graph_files/dual_total_graph_200m.graphml"
drone_list_file_path = 'graph_files/Intentions/100_flight_intention.csv'
protection_area = 30
delay_max = 60


def solve():
    trajectories = None

    # Init model and graphs
    print("Initialising graph")
    graph, graph_dual = md.init_graphs(graph_file_path, dual_path=dual_graph_path)
    raw_graph = osmnx.load_graphml(graph_file_path)
    print("Initialising model")
    model = md.init_model(graph, graph_dual, drone_list_file_path, protection_area_size=protection_area)

    current_param, current_time = None, None
    # Generate trajectories
    print("Generating trajectories")
    trajectories, trajectories_to_fn, trajectories_to_duration, fn_order = generate_trajectories.generate_trajectories(model, graph, raw_graph, graph_dual, current_param, current_time)
    # Generate intersection points
    print("Intersection points")
    horizontal_shared_nodes_list,climb_horiz_list,descent_horiz_list,climb_climb_list,descent_descent_list = generate_trajectories.generate_intersection_points(trajectories, trajectories_to_fn, model, graph, graph_dual, raw_graph)

    # Create param for problem
    nb_flights = len(list(trajectories.keys()))
    nb_trajs = sum([len(trajectories[drone_fn]) for drone_fn in trajectories.keys()])
    # print("Devrait etre egal :", nb_trajs, len(trajectories_to_fn))
    maxDelay = delay_max
    nbPt = [len(horizontal_shared_nodes_list), len(climb_horiz_list), len(descent_horiz_list), len(climb_climb_list), len(descent_descent_list)]
    d = [trajectories_to_duration[i] for i in range(nb_trajs)] # Duration of horizontal_traj_i
    mon_vol = [fn_order.index(trajectories_to_fn[i]) for i in range(nb_trajs)]
    all_pt = horizontal_shared_nodes_list + climb_horiz_list + descent_horiz_list + climb_climb_list + descent_descent_list
    k1 = [pt[0] for pt in all_pt]
    k2 = [pt[1] for pt in all_pt]
    t1 = [pt[2] for pt in all_pt]
    t2 = [pt[3] for pt in all_pt]
    sep12 = [pt[4] for pt in all_pt]
    sep21 = [pt[5] for pt in all_pt]
    print(nb_flights, nb_trajs, maxDelay, nbPt, len(d), len(mon_vol), len(all_pt))
    Afix = []
    Kfix = []
    yfix = []
    delayfix = []
    param = Param.Param(nb_flights, nb_trajs, maxDelay, nbPt, d, mon_vol, k1, k2, t1, t2, sep12, sep21, Afix, Kfix, yfix, delayfix)

    # Create Problem
    problem = PLNE.Problem(param)

    # Solve
    problem.solve()
    problem.printSolution()

    # Iterate on the deposit times left
        # Determine fixed flights and create params
        # Create param, problem, solve


solve()



