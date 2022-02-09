import matplotlib.pyplot as plt
import networkx as nx
import tools
from tools import m_displacement_to_lat_lon
from Model import init_graphs
from math import cos, radians, pi

hex_map = []
# Radius of each hexagon
hex_radius = 200  # in m
graph_file_path = "graph_files/geo_data_new/crs_epsg_32633/road_network/crs_4326_cleaned_simplified_network/cleaned_simplified.graphml"
graph_save_path = "graph_files/total_graph_"+str(hex_radius) +"m.graphml"
# Create the hexagonal grid and fix it to the constrained airspace

raw_graph = nx.read_graphml(graph_file_path)
graph = nx.Graph()
# Creating a new graph without what's not needed
for node in raw_graph.nodes:
    graph.add_node(node)
    graph.nodes[node]["x"] = float(raw_graph.nodes[node]["x"])
    graph.nodes[node]["y"] = float(raw_graph.nodes[node]["y"])
for edge in raw_graph.edges:
    graph.add_edge(edge[0], edge[1])
    graph.edges[edge[0], edge[1]]["length"] = raw_graph.edges[edge]["length"]
    graph.edges[edge[0], edge[1]]["geometry"] = raw_graph.edges[edge]["geometry"]
    # print(graph.edges[edge[0], edge[1]]["geometry"])

# (lon, lat)
north_west_x_y = (48.29509614394607, 16.23585715322775)
south_east_x_y = (48.1202024787435, 16.516030216585275)
centre = (48.206250768118295, 16.37147524175115)

y_start = north_west_x_y[0]
current_line_y = y_start
last_line_y = 0
x_start = north_west_x_y[1]
x = x_start
added_2R_last = False
start_with_r = False

while current_line_y > south_east_x_y[0]:
    current_line = []
    if start_with_r:
        new_pt = m_displacement_to_lat_lon((x_start, current_line_y), 0, hex_radius / 2)
        x = new_pt[0]
        start_with_r = False
        added_2R_last = True
    else:
        x = x_start
        start_with_r = True
        added_2R_last = False
    while x < south_east_x_y[1]:
        pt = (x, current_line_y)
        current_line.append(pt)
        if added_2R_last:
            new_pt = m_displacement_to_lat_lon(pt, 0, hex_radius)
            # print(new_pt)
            x = new_pt[0]
            added_2R_last = False
        else:
            new_pt = m_displacement_to_lat_lon(pt, 0, 2 * hex_radius)
            # print(pt, new_pt)
            x = new_pt[0]
            added_2R_last = True
    hex_map.append(current_line)
    new_pt = m_displacement_to_lat_lon(pt, -((3 ** 0.5) * hex_radius) / 2, 0)
    current_line_y = new_pt[1]

# for i in range(3):
#     print(hex_map[i])

unconstrained_graph = nx.Graph()


def get_index(line_index, node_index):
    return '{:0>3}'.format(str(line_index)) + '{:0>3}'.format(str(node_index))


line_index, node_index = 0, 0
for line in hex_map:
    node_index = 0
    for node in line:
        _idx = get_index(line_index, node_index)
        unconstrained_graph.add_node(_idx)
        unconstrained_graph.nodes[_idx]["x"] = node[0]
        unconstrained_graph.nodes[_idx]["y"] = node[1]
        node_index += 1
    line_index += 1
    # print(unconstrained_graph.nodes[node])
print(len(unconstrained_graph.nodes))
line_index, node_index = 0, 0
while line_index < len(hex_map) - 1:
    node_index = 0
    while node_index < len(hex_map[line_index]) - 1:
        if line_index == 0:
            if node_index % 2 == 0:
                unconstrained_graph.add_edge(get_index(line_index, node_index), get_index(line_index + 1, node_index))
            if node_index % 2 == 1:
                unconstrained_graph.add_edge(get_index(line_index, node_index), get_index(line_index + 1, node_index))
                unconstrained_graph.add_edge(get_index(line_index, node_index), get_index(line_index, node_index + 1))
        elif line_index % 2 == 0:
            if node_index % 2 == 0:
                unconstrained_graph.add_edge(get_index(line_index, node_index), get_index(line_index + 1, node_index))
                unconstrained_graph.add_edge(get_index(line_index, node_index), get_index(line_index - 1, node_index))
            if node_index % 2 == 1:
                unconstrained_graph.add_edge(get_index(line_index, node_index), get_index(line_index + 1, node_index))
                unconstrained_graph.add_edge(get_index(line_index, node_index), get_index(line_index - 1, node_index))
                unconstrained_graph.add_edge(get_index(line_index, node_index), get_index(line_index, node_index + 1))
        elif line_index % 2 == 1:
            if node_index % 2 == 0:
                unconstrained_graph.add_edge(get_index(line_index, node_index), get_index(line_index, node_index + 1))
        node_index += 1
    line_index += 1

# line_index, node_index = 0, 0
# while line_index < len(hex_map)-1:
#     node_index = 0
#     while node_index < len(hex_map[line_index])-1:
#         if line_index == 0:
#             if node_index % 2 == 0:
#                 unconstrained_graph.add_edge("U"+str((line_index, node_index)), "U"+str((line_index+1, node_index)))
#             if node_index % 2 == 1:
#                 unconstrained_graph.add_edge("U"+str((line_index, node_index)), "U"+str((line_index + 1, node_index)))
#                 unconstrained_graph.add_edge("U"+str((line_index, node_index)), "U"+str((line_index, node_index + 1)))
#         elif line_index % 2 == 0:
#             if node_index % 2 == 0:
#                 unconstrained_graph.add_edge("U"+str((line_index, node_index)), "U"+str((line_index + 1, node_index)))
#                 unconstrained_graph.add_edge("U"+str((line_index,node_index)), "U"+str((line_index - 1, node_index)))
#             if node_index % 2 == 1:
#                 unconstrained_graph.add_edge("U"+str((line_index,node_index)), "U"+str( (line_index + 1,node_index)))
#                 unconstrained_graph.add_edge("U"+str((line_index,node_index)), "U"+str((line_index - 1,node_index)))
#                 unconstrained_graph.add_edge("U"+str((line_index,node_index)), "U"+str((line_index,node_index + 1)))
#         elif line_index % 2 == 1:
#             if node_index % 2 == 0:
#                 unconstrained_graph.add_edge("U"+str((line_index,node_index)), "U"+str((line_index,node_index + 1)))
#         node_index += 1
#     line_index += 1


print(len(unconstrained_graph.nodes))

nodes_to_be_removed = []
idx = 0
for node in unconstrained_graph.nodes:
    # print(tools.distance(node[0], node[1], centre[1], centre[0]))
    if tools.distance(unconstrained_graph.nodes[node]["x"], unconstrained_graph.nodes[node]["y"], centre[1],
                      centre[0]) > 8000:
        nodes_to_be_removed.append(node)
for node in nodes_to_be_removed:
    unconstrained_graph.remove_node(node)

for edge in unconstrained_graph.edges:
    unconstrained_graph.edges[edge]["length"] = str(hex_radius)
    unconstrained_graph.edges[edge]["geometry"] = ("LINESTRING (" + str(
        unconstrained_graph.nodes[edge[0]]["x"]) + " " + str(unconstrained_graph.nodes[edge[0]]["y"]) + ", " + str(
        unconstrained_graph.nodes[edge[1]]["x"]) + " " + str(unconstrained_graph.nodes[edge[1]]["y"])) + ")"

nodes_to_be_removed = []
nodes_to_be_reconnected = set()
dist_mini = 500
for node1 in unconstrained_graph.nodes:
    for node2 in graph.nodes:
        # print(tools.distance(node1[0], node1[1], graph.nodes[node2]["x"], graph.nodes[node2]["y"]))
        if tools.distance(unconstrained_graph.nodes[node1]["x"], unconstrained_graph.nodes[node1]["y"],
                          graph.nodes[node2]["x"], graph.nodes[node2]["y"]) < dist_mini:
            nodes_to_be_removed.append(node1)
            for nd in unconstrained_graph.neighbors(node1):
                nodes_to_be_reconnected.add(nd)
            break
for node in nodes_to_be_removed:
    unconstrained_graph.remove_node(node)
    if node in nodes_to_be_reconnected:
        nodes_to_be_reconnected.remove(node)

# Create final total graph
total_graph = nx.Graph()
total_graph.add_nodes_from(unconstrained_graph.nodes(data=True))
total_graph.add_nodes_from(graph.nodes(data=True))
total_graph.add_edges_from(unconstrained_graph.edges(data=True))
total_graph.add_edges_from(graph.edges(data=True))
# Connect the nodes that need to be
already_connected_nodes = []
for node_to_connect in nodes_to_be_reconnected:
    closest_node = None
    min_dist = 1000000000000
    for node in graph.nodes:
        d = tools.distance(total_graph.nodes[node_to_connect]["x"], total_graph.nodes[node_to_connect]["y"],
                           graph.nodes[node]["x"], graph.nodes[node]["y"])
        if d < min_dist:
            min_dist = d
            closest_node = node
    if closest_node is not None and closest_node not in already_connected_nodes:
        total_graph.add_edge(node_to_connect, closest_node)
        total_graph.edges[(node_to_connect, closest_node)]["length"] = str(
            tools.distance(total_graph.nodes[(node_to_connect, closest_node)[0]]["x"],
                           total_graph.nodes[(node_to_connect, closest_node)[0]]["y"],
                           total_graph.nodes[(node_to_connect, closest_node)[1]]["x"],
                           total_graph.nodes[(node_to_connect, closest_node)[1]]["y"]))
        total_graph.edges[(node_to_connect, closest_node)]["geometry"] = ("LINESTRING (" + str(
            total_graph.nodes[(node_to_connect, closest_node)[0]]["x"]) + " " + str(
            total_graph.nodes[(node_to_connect, closest_node)[0]]["y"]) + ", " + str(
            total_graph.nodes[(node_to_connect, closest_node)[1]]["x"]) + " " + str(
            total_graph.nodes[(node_to_connect, closest_node)[1]]["y"])) + ")"
        # print(total_graph.edges[(node_to_connect, closest_node)]["geometry"])
        # print(total_graph.edges[(node_to_connect, closest_node)]["geometry"])
        # print( total_graph.edges[(node_to_connect, closest_node)]["length"])
        already_connected_nodes.append(closest_node)
        # if tools.edges_intersect()

osmid = 1
for edge in total_graph.edges:
    total_graph.edges[edge]["osmid"] = osmid
    osmid += 1

#
# all_points_x = []
# all_points_y = []
# for line in hex_map:
#     for pt in line:
#         all_points_x.append(pt[1])
#         all_points_y.append(pt[0])
# plt.scatter(all_points_y, all_points_x, color="grey", marker='.')
#
# all_points_x = []
# all_points_y = []
# # for node in unconstrained_graph.nodes:
# #     all_points_x.append(node[1])
# #     all_points_y.append(node[0])
# # plt.scatter(all_points_y, all_points_x)
# for node in unconstrained_graph.nodes:
#     all_points_x.append(unconstrained_graph.nodes[node]["y"])
#     all_points_y.append(unconstrained_graph.nodes[node]["x"])
# plt.scatter(all_points_y, all_points_x)
#
# all_points_x = []
# all_points_y = []
# for node in graph.nodes:
#     all_points_x.append(graph.nodes[node]["y"])
#     all_points_y.append(graph.nodes[node]["x"])
# plt.scatter(all_points_y, all_points_x, marker="*")
#
# all_points_x = []
# all_points_y = []
# for node in nodes_to_be_reconnected:
#     all_points_x.append(node[1])
#     all_points_y.append(node[0])
# plt.scatter(all_points_y, all_points_x, marker="o")
#
# for edge in unconstrained_graph.edges:
#     plt.plot([edge[0][0], edge[1][0]], [edge[0][1], edge[1][1]])
# plt.show()

all_points_x = []
all_points_y = []
for node in total_graph.nodes:
    # print(total_graph.nodes[node])
    all_points_x.append(total_graph.nodes[node]["y"])
    all_points_y.append(total_graph.nodes[node]["x"])
plt.scatter(all_points_y, all_points_x, marker="*")

for edge in total_graph.edges:
    if edge not in graph.edges:
        plt.plot([total_graph.nodes[edge[0]]["x"], total_graph.nodes[edge[1]]["x"]],
                 [total_graph.nodes[edge[0]]["y"], total_graph.nodes[edge[1]]["y"]])
plt.show()

final_graph = nx.Graph()
final_graph.graph["crs"] = "epsg:4326"
# final_graph["crs"] = "epsg:4326"

for node in total_graph.nodes:
    final_graph.add_node(node)
    final_graph.nodes[node]["x"] = str(total_graph.nodes[node]["x"])
    final_graph.nodes[node]["y"] = str(total_graph.nodes[node]["y"])

for edge in total_graph.edges:
    final_graph.add_edge(edge[0], edge[1])
    final_graph.edges[edge]["geometry"] = total_graph.edges[edge]["geometry"]
    final_graph.edges[edge]["length"] = str(total_graph.edges[edge]["length"])
    final_graph.edges[edge]["osmid"] = str(total_graph.edges[edge]["osmid"])

nx.write_graphml(final_graph, graph_save_path)
#
# with open(graph_save_path, "r") as f:
#     lines = f.readlines()
# with open(graph_save_path, "w") as f:
#     for line in lines:
#         if "<graph edgedefault"in line:
#             # f.write("  <key id=\"d0\" for=\"graph\" attr.name=\"crs\" attr.type=\"string\" />\n")
#             f.write(line)
#         if "</graph>" in line:
#             # f.write("    <data key=\"d0\">epsg:4326</data>\n")
#             f.write(line)
#             # f.write("\n")
#         else:
#             f.write(line)
#             # f.write("\n")
