import networkx as nx
import Astar2 as a2
import Graph_repr as gr
import Node as nd
import Model as md
import matplotlib.pyplot as plt
import Cluster as cl
import Drone as dr
import time
import Path as pt
import dual_graph

def run_Astar(dep, arr):
    G = md.init_graph(nx.read_graphml("graph_files\\processed_graphM2.graphml"))
    # dep = '33143898'
    # arr = '33143898'
    # c = cl.Cluster('33345332', G)
    d = dr.Drone('M600', dep, arr, 0, 'type')
    sol = a2.Astar(G, dep, arr, d)
    path = pt.Path(d.dep)
    path.set_path(sol, G)
    d.path_object = path
    print([sol[i].id for i in range(len(sol))])
    print("Total time of travel: ", sol[-1].time)

    c = cl.Cluster('264055538', (), G)
    sol2 = nx.shortest_path(G, dep, arr, 'length')
    print(sol2)
    dist = 0
    for i in range(0, len(sol2) - 1):
        dist += G.edges[sol2[i], sol2[i + 1]]['length']
    print("Cost using nx.shortest_path: ", dist)
    sol2 = [nd.Node(el) for el in sol2]

    # gr.draw_solution(G, d, show_id=False, show=False)
    # plt.savefig("solutions\\pb1_1.png".format(), dpi=1000)
    # plt.show()


def solve_clusters():
    startT = time.time()
    G = nx.read_graphml("graph_files/processed_graphM2.graphml")
    m = md.Model(G)
    m.add_drones_from_file('graph_files/drones.txt')
    flightTime = 0
    flightDistance = 0
    for d in m.droneList:
        path = a2.Astar(G, d.dep, d.arr, d, d.dep_time)
        path.set_path(path.path_object, G, d)
        path.discretize_path(5, G, d)
        path.flight_time_and_distance(G, d)
        d.path_object = path
        d.time = path.flightTime
        flightTime += path.flightTime
        flightDistance += path.flightDistance
    print('________________________')
    print("Total flight time:", flightTime)
    print("Total flight distance:", flightDistance)
    conflicts = m.find_conflicts(G)
    print('\n Initial number of conflicts: ', len(conflicts))
    compteur = 0

    while len(conflicts) != 0 and compteur < 100:
        gr.reset_color(G)

        conflicts.sort(key=lambda x: x[2])
        c = conflicts.pop(0)

        print('####################')
        print('Number of conflicts :', len(conflicts) + 1)
        print('Current conflict being solved :', c)

        d = m.droneList[c[0]]

        compteur += 1

        edge = d.find_current_edge(c[2], G)
        cluster = cl.Cluster(edge[0], (m.droneList[c[0]], m.droneList[c[1]]), G)
        cluster.find_drones(m, c[2])
        print(len(cluster.drones))

        # gr.draw_conflict(m, (m.droneList[c[0]], m.droneList[c[1]]), cluster, show_id=False, show_discretized=True, show_all_drones=True, show=False)
        # plt.savefig("solutions\\plt_sooo.pdf".format(), dpi=100)
        # plt.show()
        # # plt.savefig("solutions\\plt_{}_{}_{}.pdf".format(c[0], c[1], c[2]), dpi=100)
        # # plt.show()
        # gr.reset_color(G)

        cluster.solve_cluster(m)

        # gr.draw_conflict(m, (m.droneList[c[0]], m.droneList[c[1]]), cluster, show_id=False, show_discretized=False, show_all_drones=False, show=False)
        # plt.savefig("solutions\\plt_{}_{}_{}_resolved.pdf(".format(c[0], c[1], c[2]), dpi=100)

        conflicts = m.find_conflicts(G)

    if len(conflicts) != 0:
        print('-----------------------------------------')
        print(len(conflicts))
        print([(d.flight_number, d.path_object.hStart, d.dep_time) for d in cluster.drones])
        for d in cluster.drones:
            gr.draw_solution(G, d, show_id=False, show_discretized=True, show_time=True, show=False)
            plt.savefig("solutions\\plt_sooo_{}.pdf".format(d.flight_number), dpi=100)
            plt.show()

    print('##############################\n##############################')
    print('Final number of conflicts:', len(conflicts))
    print('Number of iterations:', compteur)
    delay = 0
    noDelay = 0
    maxDelay = 0
    flightTime = 0
    flightDistance = 0
    maxAdded = 0
    noAdded = 0
    for d in m.droneList:
        flightTime += d.path_object.flightTime
        added = d.path_object.flightTime - d.time
        if added > maxAdded:
            maxAdded = added
        if added == 0:
            noAdded += 1
        flightDistance += d.path_object.flightDistance
        delayDrone = d.path_object.hStart - d.dep_time

        for node in d.path_object.delay:
            delayDrone += d.path_object.delay[node]
        delay += delayDrone
        if delayDrone == 0:
            noDelay += 1
        if delayDrone > maxDelay:
            maxDelay = delayDrone
    print("Total flight time:", flightTime)
    print("Total flight distance:", flightDistance)
    print("Maximum overtime:", maxAdded, "and no added:", noAdded)
    print('Total delay:', delay, 'including with no delay:', noDelay, 'and max delay:', maxDelay)
    print('Time taken by algorithm:', time.time() - startT, 's')
    for d in m.droneList:
        print('Delay at departure for', d.flight_number, ':', d.path_object.hStart - d.dep_time)
    return len(conflicts)


def solve_clusters_with_dual():
    """Solve the pathing problem for the drones of the specified file on the graph using the cluster method and the A*
    algorithm on a dual graph of the given graph, which takes into account the influence of turns on drones speed"""
    start_time = time.time()

    # 1 Initialise the model and the graph (normal and dual) from files
    model, graph, graph_dual = init_model(graph_file_path, drone_list_file_path)

    # 2 Compute the shortest path for each drone using the A* algorithm on the dual graph
    print("Computing shortest path for each drone")
    model, initial_total_flight_time, initial_total_flight_distance = compute_all_shortest_paths(model, graph, graph_dual)
    print('________________________')
    print("Total flight time:", initial_total_flight_time)
    print("Total flight distance:", initial_total_flight_distance)

    # 3 Finding conflicts
    conflicts = model.find_conflicts(graph)
    print('\n Initial number of conflicts: ', len(conflicts))

    # 4 Solving
    iteration_count = 0
    while len(conflicts) != 0 and iteration_count < max_iteration:
        iteration_count += 1
        gr.reset_color(graph)
        # Sorting conflicts by time to solve the early ones first
        conflicts.sort(key=lambda x: x[2])
        current_conflict = conflicts.pop(0)
        print('####################')
        # Display number of conflicts (+1 to take into account the one being solved that was popped)
        print('Number of conflicts :', len(conflicts) + 1)
        print('Current conflict being solved :', current_conflict)
        drone = model.droneList[current_conflict[0]]
        edge = drone.find_current_edge(current_conflict[2], graph)
        cluster = cl.Cluster(edge[0], (model.droneList[current_conflict[0]], model.droneList[current_conflict[1]]), graph)
        cluster.find_drones(model, current_conflict[2])
        print(len(cluster.drones))
        cluster.solve_cluster(model)
        conflicts = model.find_conflicts(graph)
    if len(conflicts) != 0:
        print('-----------------------------------------')
        print(len(conflicts))
        print([(d.flight_number, d.path_object.hStart, d.dep_time) for d in cluster.drones])
    print('##############################\n##############################')
    print('Final number of conflicts:', len(conflicts))
    print('Number of iterations:', iteration_count)

    # 5 Compute and display Results
    final_metrics_dict = compute_and_display_results(model, start_time)

    # 6 Save solutions
    print("Drawing solutions")
    for drone in model.droneList:
        gr.draw_solution(graph, drone, show_id=False, show_discretized=True, show_time=True, show=False)
        plt.savefig("solutions/plt_sol_{}.png".format(drone.flight_number), dpi=1000)
        plt.close()

    return len(conflicts)



def seq_solve():
    startT = time.time()
    G = nx.read_graphml("graph_files\\processed_graphM2.graphml")
    m = md.Model(G)
    m.add_drones_from_file('graph_files\\drones.txt')
    c = cl.Cluster('33144572', (m.droneList[0], m.droneList[1]), G)
    c.drones = m.droneList.copy()
    c.edgesList = []
    c.nodesList = []
    for edge in G.edges:
        c.edgesList.append(edge)
    for node in G.nodes:
        c.nodesList.append(node)

    constraintsNodes = {}
    constraintsEdges = {}
    flightTime = 0
    flightDistance = 0
    noSol = 0
    for d in c.drones:
        print(d.flight_number)
        path = None
        delay = -5
        while path == None:
            delay += 5
            path = a2.Astar(G, d.dep, d.arr, d, d.dep_time + delay, constraintsNodes, constraintsEdges)
        if path != None:
            path.delay[d.dep] = delay
            print(path.delay)
            path.add_path(path.path, G, d)
            path.discretize_path(5, G, d)
            path.flight_time_and_distance(G, d)
            d.path_object = path
            d.time = path.flightTime
            constraintsNodes, constraintsEdges = c.add_constraints(constraintsNodes, constraintsEdges, d, path)
            flightTime += path.flightTime
            flightDistance += path.flightDistance
        else:
            noSol += 1
    print('________________________')
    print("Total flight time:", flightTime)
    print("Total flight distance:", flightDistance)
    print("Drones without a solution:", noSol)
    print("Number of conflicts:", len(m.find_conflicts(G)))

    gr.draw_conflict(m, (m.droneList[23], m.droneList[38]), c, show_id=False, show_discretized=True,
                     show_all_drones=False, show=False)
    plt.savefig("solutions/plt_sooo.pdf".format(), dpi=100)
    plt.show()
    gr.reset_color(G)



    # def solve_cluster_delay(self, model):
    #     perms = list(itertools.permutations(self.drones))
    #     best_time = 10000000
    #
    #     def new_path(drone, constraints_nodes, constraints_edges):
    #         found = False
    #         i = 0
    #         drone_path = drone.path
    #         while not found:
    #             if drone_path.path[i] in self.nodesList:
    #                 dep = drone_path.path[i]
    #                 found = True
    #             else:
    #                 i += 1
    #         pathSol = None
    #         iDict = list(drone_path.pathDict.values()).index(dep)
    #         timeDep = list(drone_path.pathDict.keys())[iDict]
    #         delay = -5
    #         while pathSol == None and delay < 60:
    #             delay += 5
    #             pathSol = a2.Astar(model.graph, dep, drone.arr, drone, timeDep + delay, constraints_nodes, constraints_edges)
    #
    #         newSol = None
    #
    #         newPath = drone.path.path[:i].copy() + pathSol.path.copy()
    #         try:
    #             pDelay = drone.path.delay[dep]
    #         except:
    #             pDelay = 0
    #         try:
    #             ppDelay = drone.path.previousPath.delay[dep]
    #         except:
    #             ppDelay = 0
    #         try:
    #             previousPath = drone.path.previousPath.path
    #         except:
    #             previousPath = []
    #
    #         if (drone.path.path != newPath or (drone.path.path == newPath and pDelay != delay)) and (newPath != previousPath or (newPath == previousPath and ppDelay != delay)):
    #             try:
    #                 hDep = drone.path.hStart
    #             except:
    #                 hDep = drone.hDep
    #             newSol = pt.Path(hDep, newPath.copy())
    #             newSol.delay = drone.path.delay.copy()
    #             for node in newSol.delay.copy():
    #                 if node not in newPath:
    #                     newSol.delay.pop(node)
    #             if delay != 0:
    #                 try:
    #                     newSol.delay[dep] += delay
    #                 except KeyError:
    #                     newSol.delay[dep] = delay
    #             if dep == drone.dep and delay != 0:
    #                 newSol.hStart += delay
    #                 newSol.delay.pop(dep)
    #             newSol.previousPath = drone.path
    #             newSol.add_path(newPath.copy(), model.graph, drone)
    #             newSol.discretize_path(5, model.graph, drone)
    #             newSol.flight_time_and_distance(model.graph, drone)
    #             newSol.flightTime += newSol.hStart - drone.hDep
    #
    #         constraints_nodes, constraints_edges = self.add_constraints(constraints_nodes, constraints_edges, drone, newSol)
    #
    #         return constraints_nodes, constraints_edges, newSol, delay
    #
    #     for dronesList in perms:
    #         constraintsNodes = self.initial_constraint_nodes
    #         constraintsEdges = self.initial_constraint_edges
    #     # constraintsNodes = {}
    #     # constraintsEdges = {}
    #         pathsChanged = []
    #         pathsUnchanged = []
    #         for d in dronesList:
    #             constraintsNodes, constraintsEdges, path, delay = new_path(d, constraintsNodes, constraintsEdges)
    #             if path != None :
    #                 # d.path = path
    #                 pathsChanged.append(path)
    #             else:
    #                 pathsUnchanged.append(d.path)
    #
    #         totalTime = 0
    #         if len(pathsChanged) >= 1:
    #             for p in pathsChanged:
    #                 totalTime += p.flightTime
    #             for p in pathsUnchanged:
    #                 totalTime += p.flightTime
    #         else:
    #             totalTime = best_time + 10
    #         if totalTime <= best_time:
    #             best_time = totalTime
    #             bestPerm = dronesList
    #
    #     constraintsNodes = {}
    #     constraintsEdges = {}
    #     for d in bestPerm:
    #         constraintsNodes, constraintsEdges, path, delay = new_path(d, constraintsNodes, constraintsEdges)
    #         if path != None:
    #             d.path = path
    #
    # def solve_cluster(self, model):
    #     perms = list(itertools.permutations(self.drones))
    #     print("number of permutations ", len(perms))
    #     bestTime = 100000
    #
    #     def new_path(drone, constraint_nodes, constraints_edges):
    #         found = False
    #         i = 0
    #         while not found:
    #             if drone.path.path[i] in self.nodesList:
    #                 dep = drone.path.path[i]
    #                 found = True
    #             else:
    #                 i += 1
    #
    #         i = list(drone.path.pathDict.values()).index(dep)
    #         timeDep = list(drone.path.pathDict.keys())[i]
    #
    #         path_solution = a2.Astar(model.graph, dep, drone.arr, drone, timeDep, constraint_nodes, constraints_edges)
    #         new_solution = None
    #         if path_solution is not None:
    #             if not drone.path.previousPath == []:
    #                 previous_path = drone.path.previousPath.path
    #             else :
    #                 previous_path = []
    #             # print(drone)
    #             # previous_path = drone.path.previousPath.path
    #             new_path = drone.path.path[:i].copy() + path_solution.path.copy()
    #             if drone.path.path != new_path and new_path != previous_path:
    #                 new_solution = pt.Path(drone.hDep, new_path.copy())
    #                 new_solution.previousPath = drone.path
    #                 new_solution.add_path(new_path.copy(), model.graph, drone)
    #                 new_solution.discretize_path(5, model.graph, drone)
    #                 new_solution.flight_time_and_distance(model.graph, drone)
    #         constraint_nodes, constraints_edges = self.add_constraints(constraint_nodes, constraints_edges, drone, new_solution)
    #         return constraint_nodes, constraints_edges, new_solution
    #
    #     for dronesList in perms:
    #         constraintsNodes = {}
    #         constraintsEdges = {}
    #         pathsChanged = []
    #         pathsUnchanged = []
    #         for d in dronesList:
    #             constraintsNodes, constraintsEdges, path = new_path(d, constraintsNodes, constraintsEdges)
    #             if path != None:
    #                 pathsChanged.append(path)
    #             else:
    #                 pathsUnchanged.append(d.path)
    #         totalTime = 0
    #         if len(pathsChanged) != 0:
    #             for p in pathsChanged:
    #                 totalTime += p.flightTime
    #             for p in pathsUnchanged:
    #                 totalTime += p.flightTime
    #         else:
    #             totalTime = bestTime + 10
    #         if totalTime <= bestTime:
    #             bestTime = totalTime
    #             bestPerm = dronesList
    #
    #     constraintsNodes = {}
    #     constraintsEdges = {}
    #     for d in bestPerm:
    #         constraintsNodes, constraintsEdges, path = new_path(d, constraintsNodes, constraintsEdges)
    #         if path != None:
    #             d.path = path
    #         # print(" Final paths : ", d.path.path)