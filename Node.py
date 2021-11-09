import math
# import networkx as nx
# import Model as md


class Node:
    def __init__(self, name, parent=None):
        self.id = name
        self.cost = math.inf
        self.heuristic = math.inf
        self.parent = parent
        self.time = math.inf
    
    def f(self):
        """Returns the sum of the heuristic and the cost of the associated node. """
        return self.heuristic + self.cost
    
    def path(self):
        """Finds and returns the path to the node from the start."""
        path = [self]
        node = self.parent
        while node is not None:
            path.append(node)
            node = node.parent
        path.reverse()
        return path
    
    def dist_to_node(self, node, graph):
        """Allows to compute the shortest distance to another node.
        Used to find the heuristic of a node to the end."""
        earth_radius = 6371009

        x1, y1 = graph.nodes[self.id]['x']*math.pi/180, graph.nodes[self.id]['y']*math.pi/180
        x2, y2 = graph.nodes[node]['x']*math.pi/180, graph.nodes[node]['y']*math.pi/180
        try:
            return math.acos(math.sin(y1)*math.sin(y2)+math.cos(y1)*math.cos(y2)*math.cos(x1-x2)) * earth_radius
        except ValueError:
            return math.acos(math.cos(90-y1)*math.cos(90-y2)+math.sin(90-y1)*math.sin(90-y2)*math.cos(x1-x2)) * earth_radius
