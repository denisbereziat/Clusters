import networkx as nx
import tools


def create_digraph_with_source_and_terminal(graph):
    digraph = nx.DiGraph(graph)
    for node in graph.nodes():
        digraph.add_node("S" + node)
        digraph.add_edges_from([("S" + node, node, {'length': 0})])
        digraph.add_node(node + "T")
        digraph.add_edges_from([(node, node + "T", {'length': 0})])
    return digraph


def create_dual(graph, turn_cost_function):
    """Create a Dual Graph from a directed graph with added weights depending on the turns"""
    digraph = create_digraph_with_source_and_terminal(graph)
    dual_graph = nx.DiGraph()
    # Each edge in the primal graph creates a node in the dual graph
    for edge in digraph.edges():
        dual_graph.add_node(edge)
    # Adding all the required edges between connected nodes
    for node1 in dual_graph.nodes():
        for node2 in dual_graph.nodes():
            if node1[1] == node2[0]:
                if node1[1] == node2[0] and node1[0] == node2[1]:
                    continue
                dual_graph.add_edge(node1, node2)
                # Adding the turn cost if it isn't a starting or ending edge (node starting with S or ending with T)
                if node2[1][-1] == "T" or node1[0][0] == "S":
                    total_turn_cost, pre_turn_cost, post_turn_cost, is_turn = (0, 0, 0, False)
                    angle = 0
                else:
                    total_turn_cost, pre_turn_cost, post_turn_cost, is_turn = turn_cost_function(digraph.nodes[node1[0]],digraph.nodes[node1[1]],digraph.nodes[node2[1]])
                    x1, y1 = float(digraph.nodes[node1[0]]["y"]), float(digraph.nodes[node1[0]]["x"])
                    x2, y2 = float(digraph.nodes[node1[1]]["y"]), float(digraph.nodes[node1[1]]["x"])
                    x3, y3 = float(digraph.nodes[node2[1]]["y"]), float(digraph.nodes[node2[1]]["x"])
                    # v1 = (x2 - x1, y2 - y1)
                    # v2 = (x3 - x2, y3 - y2)
                    # angle = tools.angle_btw_vectors(v1, v2)
                    if tools.is_in_unconstrained(node1[0]) and tools.is_in_unconstrained(node1[1]) and tools.is_in_unconstrained(node2[1]):
                        angle = 0
                        is_turn = False
                    else:
                        angle = tools.angle_btw_vectors((x1, y1), (x2, y2), (x3, y3))
                edge_length = float(digraph.edges[node1[0], node1[1]]["length"])
                # le turn cost est ajouté de base il faut retirer le post_cost pour avoir le cout au moment ou on passe le node
                dual_graph.edges[node1, node2]["length"] = edge_length + total_turn_cost
                dual_graph.edges[node1, node2]["total_turn_cost"] = total_turn_cost
                dual_graph.edges[node1, node2]["post_turn_cost"] = post_turn_cost
                dual_graph.edges[node1, node2]["pre_turn_cost"] = pre_turn_cost
                dual_graph.edges[node1, node2]["is_turn"] = is_turn
                dual_graph.edges[node1, node2]["angle"] = float(angle)
    for node in dual_graph.nodes():
        if node[1][-1] == "T":
            dual_graph.nodes[node]["x"] = float(digraph.nodes[node[0]]["x"])
            dual_graph.nodes[node]["y"] = float(digraph.nodes[node[0]]["y"])
        else:
            dual_graph.nodes[node]["x"] = float(digraph.nodes[node[1]]["x"])
            dual_graph.nodes[node]["y"] = float(digraph.nodes[node[1]]["y"])
    return dual_graph
