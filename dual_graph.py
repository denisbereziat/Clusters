import networkx as nx


def create_digraph_with_source_and_terminal(graph):
    digraph = nx.DiGraph(graph)
    for node in graph.nodes():
        digraph.add_node("S" + node)
        digraph.add_edges_from([("S" + node, node, {'length': 0})])
        digraph.add_node(node + "T")
        digraph.add_edges_from([(node, node + "T", {'length': 0})])
    return digraph


def create_dual(model, turn_cost_function):
    """Create a Dual Graph from a directed graph with added weights depending on the turns"""
    digraph = create_digraph_with_source_and_terminal(model.graph)
    dual_graph = nx.DiGraph()
    for edge in digraph.edges():
        dual_graph.add_node(edge)
    for node1 in dual_graph.nodes():
        for node2 in dual_graph.nodes():
            # This should prevent edges allowing to turn back
            if node1[1] == node2[0]: # and node1[0] != node2[1]:
                dual_graph.add_edge(node1, node2)
                # Adding the turn cost if it isn't a starting or ending edge (node starting with S or ending with T)
                if node2[1][-1] == "T" or node1[0][0] == "S":
                    total_turn_cost, pre_turn_cost, post_turn_cost = (0, 0, 0)
                else:
                    total_turn_cost, pre_turn_cost, post_turn_cost = turn_cost_function(digraph.nodes[node1[0]],
                                                                                        digraph.nodes[node1[1]],
                                                                                        digraph.nodes[node2[1]])
                edge_length = float(digraph.edges[node1[0], node1[1]]["length"])
                # le turn cost est ajouté de base il faut retirer le post_cost pour avoir le cout au moment ou on passe le node
                dual_graph.edges[node1, node2]["length"] = edge_length + total_turn_cost
                dual_graph.edges[node1, node2]["total_turn_cost"] = total_turn_cost
                dual_graph.edges[node1, node2]["post_turn_cost"] = post_turn_cost
                dual_graph.edges[node1, node2]["pre_turn_cost"] = pre_turn_cost
    for node in dual_graph.nodes():
        if node[1][-1] == "T":
            dual_graph.nodes[node]["x"] = float(digraph.nodes[node[0]]["x"])
            dual_graph.nodes[node]["y"] = float(digraph.nodes[node[0]]["y"])
        else:
            dual_graph.nodes[node]["x"] = float(digraph.nodes[node[1]]["x"])
            dual_graph.nodes[node]["y"] = float(digraph.nodes[node[1]]["y"])
    return dual_graph
