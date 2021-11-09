import matplotlib.pyplot as plt


COLOR_NORMAL = 'red'
COLOR_PATH = 'blue'
COLOR_START_FINISH = 'black'


def draw_solution(graph, drone, show_id=True, show_discretized=False, show_time=True, show=True):
    color_graph(graph, drone.path_object, drone.dep, drone.arr, show_id=show_id)
    draw_with_plt(graph, show_id)
    if show_discretized:
        draw_discretized_path(drone, show_time)
    if show:
        plt.show()
        

def draw_conflict(model, dronesInConflict, cluster, show_id=True, show_discretized=False, show_time=True,
                  show_all_drones=False, show=True):
    if show_all_drones:
        for drone in cluster.drones:
            if drone not in dronesInConflict:
                draw_discretized_path(drone, show_time, node_color='purple')
            
    for drone in dronesInConflict:
        color_graph(model.graph, drone.path_object.path_object, drone.dep, drone.arr, cluster, show_id)
        draw_with_plt(model.graph, show_id)
        if show_discretized:
            draw_discretized_path(drone, show_time)
    
    if show:
        plt.show()
    
    
def color_graph(graph, sol, dep, arr, cluster=None, show_id=True):
    graph.nodes[dep]['color'] = COLOR_START_FINISH
    graph.nodes[arr]['color'] = COLOR_START_FINISH
    pos={}
    nodesColor=[]
    for node in graph.nodes:
        pos[node]= (float(graph.nodes[node]['x']), float(graph.nodes[node]['y']))
        try:
            nodesColor.append(graph.nodes[node]['color'])
        except Exception:
            graph.nodes[node]['color'] = COLOR_NORMAL
            nodesColor.append(COLOR_NORMAL)
    edgesColor=[]
    for edge in graph.edges:
        try:
            edgesColor.append(graph.edges[edge]['color'])
        except Exception:
            graph.edges[edge]['color'] = COLOR_NORMAL
            edgesColor.append(COLOR_NORMAL)    
    

def draw_with_plt(graph, show_id):
    for el in graph.edges:
        coord = graph.edges[el]['geometry']
        Xedges = []
        Yedges = []
        for i in range(0, len(coord)-1, 2):
            Xedges.append(coord[i])
            Yedges.append(coord[i+1])
        plt.plot(Xedges, Yedges, color=graph.edges[el]['color'], zorder=-1, linewidth=2)
    
    for el in graph.nodes:
        plt.scatter(graph.nodes[el]['x'], graph.nodes[el]['y'], 10, color=graph.nodes[el]['color'], zorder=1)
        if show_id:
            plt.text(graph.nodes[el]['x'], graph.nodes[el]['y'], el, fontsize=3.5)
    plt.axis('off')
    
    
def draw_discretized_path(drone, show_time=True, node_color='green'):
    for t in drone.path_object.path_dict_discretized:
        plt.scatter(drone.path_object.path_dict_discretized[t][0], drone.path_object.path_dict_discretized[t][1], 2,
                    color=node_color, zorder=3)
        if show_time:
            plt.text(drone.path_object.path_dict_discretized[t][0], drone.path_object.path_dict_discretized[t][1], str(t), size=2)
    
    
def reset_color(graph):
    for el in graph.edges:
        graph.edges[el]['color'] = COLOR_NORMAL
    for el in graph.nodes:
        graph.nodes[el]['color'] = COLOR_NORMAL
        