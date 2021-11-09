import Drone as dr
import math


class Model:
    def __init__(self, graph, protection_area, dt=5, drones=[]):
        self.graph = init_graph(graph)
        self.graph_dual = None
        self.time = 0   # keep track of the time in seconds
        self.timeInterval = dt
        self.droneList = drones
        self.protection_area = protection_area

    def add_drone(self, drone):
        self.droneList.append(drone)

    def set_graph_dual(self, graph_dual):
        self.graph_dual = graph_dual

    def add_drones_from_file(self, filename, time):
        time_in_seconds = float(time[0:2])*3600 + float(time[3:5])*60 + float(time[6:8])
        with open(filename, 'r') as f:
            for line in f:
                line = line.strip().split('\t')
                coordDep = line[4].strip('()').split(',')
                coordArr = line[5].strip('()').split(',')
                dep = closest_node(float(coordDep[1]), float(coordDep[0]), self.graph)
                arr = closest_node(float(coordArr[1]), float(coordArr[0]), self.graph)
                deposit_time_in_seconds = float(line[0][0:2])*3600 + float(line[0][3:5])*60 + float(line[0][6:8])
                # print("Deposit time :", line)
                # print("Deposit time :", deposit_time_in_seconds)
                hDep = float(line[3][0:2])*3600 + float(line[3][3:5])*60 + float(line[3][6:8])
                drone = dr.Drone(line[1], dep, arr, hDep, line[2])
                if deposit_time_in_seconds <= time_in_seconds:
                    self.add_drone(drone)

    def find_conflicts(self, graph):
        """Detect conflicts on the graph by using the distances between each drones at any time"""
        conflict_list = []
        for i, drone in enumerate(self.droneList):
            if i != len(self.droneList)-1:
                for j in range(i+1, len(self.droneList)):
                    t_edge = drone.find_conflict_on_edge(self.droneList[j], graph)
                    t_node = drone.find_conflict_on_node(self.droneList[j], self)
                    if t_edge is not None:
                        if t_node is not None:
                            if t_edge <= t_node:
                                conflict_list.append((i, j, t_edge))
                            else:
                                conflict_list.append((i, j, t_node))
                    elif t_node is not None:
                        conflict_list.append((i, j, t_node))
        return conflict_list

    def open_all_nodes(self):
        for edge in self.graph.edges:
            self.graph.edges[edge]['open'] = True


def init_graph(graph):
    for edge in graph.edges:
        edgeDict = graph.edges[edge]
        edgeDict['open'] = True
        edgeDict['length'] = float(edgeDict['length'])
        edgeDict['geometry'] = edgeDict['geometry'].strip(')').split('(')[1].split()
        for i, coord in enumerate(edgeDict['geometry']):
            edgeDict['geometry'][i] = float(coord.strip(','))
        # graph.edges[edge]['maxspeed'] = float(graph.edges[edge]['maxspeed'])
    for node in graph.nodes:
        nodeDict = graph.nodes[node]
        nodeDict['x'] = float(nodeDict['x'])
        nodeDict['y'] = float(nodeDict['y'])
    return graph        
                

def closest_node(x, y, graph):
    '''For a given set of coordinates, gives the closest node in the graph.'''
    closestNode = None
    minDist = 500000
    
    for node in graph.nodes():
        dist = math.sqrt((x-graph.nodes[node]['x'])**2+(y-graph.nodes[node]['y'])**2)
        if dist == 0:
            return node
        if dist < minDist:
            minDist = dist
            closestNode = node
    
    return closestNode


