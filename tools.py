import math
import pyproj
from shapely.geometry import LineString
import matplotlib.pyplot as plt

def hash_graph(graph, resolution):
    # TODO
    # hash_nodes = [[[] for i in range(resolution)] for i in range(resolution)]
    hash_nodes = dict()
    hash_edges = dict()
    edge_hash_tab=[]
    max_y, min_y = -1000, 1000
    max_x, min_x = -1000, 1000
    for node in graph.nodes:
        x = graph.nodes[node]["x"]
        y = graph.nodes[node]["y"]
        max_x = max(max_x, x)
        min_x = min(min_x, x)
        max_y = max(max_y, y)
        min_y = min(min_y, y)
    # for edge in graph.edges:
    #     node1 = edge[0]
    #     node2 = edge[1]
    #     x_middle = (graph.nodes[node1]["x"] + graph.nodes[node2]["x"]) /2
    #     y_middle = (graph.nodes[node1]["y"] + graph.nodes[node2]["y"]) /2
    #     middle_coords = (x_middle, y_middle)
    #     max_x = max(max_x, x_middle)
    #     min_x = min(min_x, x_middle)
    #     max_y = max(max_y, y_middle)
    #     min_y = min(min_y, y_middle)
    #     edge_hash_tab.append([edge, middle_coords])
    x_step = round((max_x-min_x) / resolution, 8)
    y_step = round((max_y-min_y) / resolution, 8)
    # print("max", max_x, max_y)
    # print("min", min_x, min_y)
    # print("step :", x_step, y_step)

    for node in graph.nodes:
        h = hash_pos(graph.nodes[node]["x"], graph.nodes[node]["y"], min_x, min_y, x_step, y_step)
        if h in hash_nodes:
            hash_nodes[h].append(node)
        else:
            hash_nodes[h] = [node]

    for edge in graph.edges:
        node1 = edge[0]
        node2 = edge[1]
        x_middle = (graph.nodes[node1]["x"] + graph.nodes[node2]["x"]) / 2
        y_middle = (graph.nodes[node1]["y"] + graph.nodes[node2]["y"]) / 2
        h = hash_pos(x_middle, y_middle, min_x, min_y, x_step, y_step)
        if h in hash_edges:
            hash_edges[h].append(edge)
        else:
            hash_edges[h] = [edge]
    return hash_nodes, hash_edges, min_x, min_y, x_step, y_step, resolution


def hash_pos(x, y, min_x, min_y, step_x, step_y):
    x_pos = (x - min_x) // step_x
    y_pos = (y - min_y) // step_y
    return x_pos, y_pos


def extract_deposit_times(filepath):
    """Extract just the flight plans deposit times"""
    deposit_times_list = []
    with open(filepath) as f:
        for line in f.readlines():
            deposit_time = line[0:8]
            if deposit_time not in deposit_times_list:
                deposit_times_list.append(deposit_time)
    # print(deposit_times_list)
    for i in range(len(deposit_times_list)):
        l = deposit_times_list[i].split(':')
        deposit_time_in_s = 3600 * int(l[0]) + 60 * int(l[1]) + int(l[2])
        deposit_times_list[i] = deposit_time_in_s
    # print(deposit_times_list)
    return deposit_times_list


def find_closest_node_with_hash(x, y, hash_map, min_x, min_y, step_x, step_y, resolution, graph):
    list = find_list_of_closest_with_hash(x, y, hash_map, min_x, min_y, step_x, step_y, resolution)
    return find_closest_node_in_list(x, y, list, graph)


def find_list_of_closest_with_hash(x, y, hash_map, min_x, min_y, step_x, step_y, resolution):
    h = hash_pos(x, y, min_x, min_y, step_x, step_y)
    possible_closest = []
    found = False
    radius = 1
    while not found:
        if radius > resolution:
            raise Exception
        for x in range(-radius, radius+1):
            for y in range(-radius, radius+1):
                _h = (h[0] + x, h[1] + y)
                if _h in hash_map:
                    # TODO ON AJOUTE DES DOUBLONS LA
                    possible_closest += hash_map[_h]
                    found = True
        radius += 1
    return possible_closest


def find_closest_node_in_list(x, y, nodes_list, graph):
    closest = None
    smallest_dst = math.inf
    for node in nodes_list:
        dist = distance(x, y, graph.nodes[node]["x"], graph.nodes[node]["y"])
        if dist < smallest_dst:
            closest = node
            smallest_dst = dist
    if closest is None:
        print("no node found")
        raise Exception
    else:
        return closest


def find_closest_edge_in_list(x, y, edge_list, graph):
    # TODO AJOUTER UNE DIST MINI POUR SAVOIR SI DANS UNCONSTRAINED OU CONSTRAIND ABD(dist) < precision
    # PARCE QUE SI ON EST TRES PROCHE DE l'ARRETE ON VEUT FAIRE COMME CONSTRAINED ET SINON  COMME UNCONSTRAUNED
    closest = None
    smallest_dist = math.inf
    for edge in edge_list:
        node1 = edge[0]
        node2 = edge[1]
        x_middle = (graph.nodes[node1]["x"] + graph.nodes[node2]["x"]) / 2
        y_middle = (graph.nodes[node1]["y"] + graph.nodes[node2]["y"]) / 2
        dist = distance(x, y, x_middle, y_middle)
        if dist < smallest_dist:
            closest = edge
            smallest_dist = dist
    if closest is None:
        print("no node found")
        raise Exception
    else:
        return closest, smallest_dist


def intersection(interval1, interval2):
    if interval2[0] > interval1[1] or interval1[0] > interval2[1]:
        return None
    else:
        return [max([interval1[0], interval2[0]]), min(interval1[1], interval2[1])]


def angle_btw_vectors(pt1, pt2, pt3):
    geodesic = pyproj.Geod(ellps='WGS84')
    fwd_azimuth1, back_azimuth1, _distance = geodesic.inv(pt1[1], pt1[0], pt2[1], pt2[0])
    fwd_azimuth2, back_azimuth2, _distance = geodesic.inv(pt2[1], pt2[0], pt3[1], pt3[0])
    angle = fwd_azimuth2 - fwd_azimuth1
    # angle = bearing(pt1, pt2) - bearing(pt2, pt3)
    # angle = angle*360/(2*math.pi)
    angle = abs(angle)
    if angle > 180:
        angle = 360 - 180
    return angle


def dot(v1, v2):
    return v1[0] * v2[0] + v1[1] * v2[1]


def distance(x1, y1, x2, y2):
    # x1, y1 = x1*math.pi/180, y1*math.pi/180
    # x2, y2 = x2*math.pi/180, y2*math.pi/180
    # try:
    #     dist =  math.acos(math.sin(y1)*math.sin(y2)+math.cos(y1)*math.cos(y2)*math.cos(x1-x2))* 6371009
    # except ValueError:
    #     dist = math.acos(math.cos(90-y1)*math.cos(90-y2)+math.sin(90-y1)*math.sin(90-y2)*math.cos(x1-x2))* 6371009
    # print("dist")
    # print(dist)
    # g = pyproj.Geod(ellps='clrk66')
    g = pyproj.Geod(ellps="WGS84")
    dist = g.line_lengths([x1,x2], [y1,y2])[0]
    # dist = g.inv(x1,y1,x2,y2)[2]
    # g = pyproj.Geod(ellps="WGS84")
    # print(dist)
    return dist


def edges_intersect(edge1, edge2, graph):
    l1 = LineString((graph.nodes[edge1[0]]["x"], graph.nodes[edge1[0]]["y"]), (graph.nodes[edge1[1]]["x"], graph.nodes[edge1[1]]["y"]))
    l2 = LineString((graph.nodes[edge2[0]]["x"], graph.nodes[edge2[0]]["y"]), (graph.nodes[edge2[1]]["x"], graph.nodes[edge2[1]]["y"]))
    return l1.intersects(l2)


def scatter_graph(graph, marker='+', color='grey'):
    x = [graph.nodes[node]["x"] for node in graph.nodes]
    y = [graph.nodes[node]["y"] for node in graph.nodes]
    plt.scatter(x,y, marker=marker, color=color)


def plot_traj(traj, graph, marker='o', color='red'):
    x = [graph.nodes[node]["x"] for node in traj]
    y = [graph.nodes[node]["y"] for node in traj]
    plt.plot(x,y, marker=marker, color=color)


def m_displacement_to_lat_lon(pt, dn, de):
    lat = pt[1]
    lon = pt[0]

    R = 6378137

    dLat = dn/R
    dLon = de/(R*math.cos(math.pi*lat/180))

    latO = lat + dLat * 180/math.pi
    lonO = lon + dLon * 180/math.pi

    return lonO, latO


def point_on_edge(x, y, edge, graph):
    def line_eq(p1, p2):
        a = (p1[1]-p2[1])/(p1[0]-p2[0])
        b = p1[1] - a*p1[1]
        f = lambda x: a*x+b
        return f
    geometry = graph.edges[edge]['geometry']
    for i in range(len(geometry)-1, 2):
        f = line_eq((geometry[i],geometry[i+1]), (geometry[i+2],geometry[i+3]))
        if abs(f(x)-y) <= 0.0001:
            return True
    return False
