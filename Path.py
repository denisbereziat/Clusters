import tools as tools
import networkx


class Path:
    def __init__(self, hStart, path=None):
        # Path contain the pth as a list of node
        if path is None:
            path = []
        self.path = path
        # Previous path is used to store the last path that was taken
        # TODO ajouter un set path pour forcer la mise a jour de previous path aussi
        self.previous_path = []
        # Path dict contain a path with the time as keys and the node as value
        self.path_dict = {}
        # The discretized version of the path with time as key and position x,y as value
        self.path_dict_discretized = {}
        self.delay = {}
        self.hStart = hStart
        # TODO ça va pas du tout ça il faut supprimer ces attributs et laisser que la methode qui calcule
        self.flightTime = 0
        self.flightDistance = 0

    def add_path(self, new_path, graph, graph_dual, drone):
        """Adds a new path to the drone, in terms of waypoints (self.path) and waypoints
        and timestamps (self.pathDict)"""
        self.path_dict = {}
        self.path = new_path
        t = self.hStart
        self.path_dict[t] = new_path[0]
        # TODO implement case where path length = 2
        for node_index in range(1, len(new_path)-1):
            # print(new_path)
            edge_dual = ((new_path[node_index-1], new_path[node_index]),
                         (new_path[node_index], new_path[node_index+1]))
            # print(edge_dual)
            # print(graph_dual.edges)
            # Determine travel cost to the node (remove the post turn cost)
            # print(edge_dual)
            try:
                cost = graph_dual.edges[edge_dual]["length"] - graph_dual.edges[edge_dual]["post_turn_cost"]
            except Exception:
                print(edge_dual)
                raise Exception
            t += cost/drone.speed
            self.path_dict[t] = new_path[node_index]
            # Add back the post turn cost
            t += graph_dual.edges[edge_dual]["post_turn_cost"]/drone.speed
        # Last node :
        cost = graph.edges[new_path[-2], new_path[-1]]["length"]
        t += cost/drone.speed
        self.path_dict[t] = new_path[-1]

        # for i in range(len(new_path) - 1):
        #     if new_path[i] in self.delay:
        #         self.path_dict[t] = new_path[i]
        #         t += self.delay[new_path[i]]
        #     self.path_dict[t] = new_path[i]
        #     t += graph.edges[new_path[i], new_path[i + 1]]['length'] / drone.speed
        # self.path_dict[t] = new_path[-1]
        
    def flight_time_and_distance(self, graph, drone):
        self.flightDistance = 0
        for i in range(len(self.path)-1):
            self.flightDistance += graph.edges[self.path[i], self.path[i+1]]['length']
        self.flightTime = self.flightDistance / drone.speed
        for node in self.delay:
            self.flightTime += self.delay[node]
            
    def discretize_path(self, dt, graph, drone):
        loop = True 
        
        x0, y0 = graph.nodes[drone.dep]['x'], graph.nodes[drone.dep]['y']
        xEnd, yEnd = graph.nodes[drone.arr]['x'], graph.nodes[drone.arr]['y']
        
        edgeIndex = 0
        edge = [self.path[edgeIndex], self.path[edgeIndex+1]]
        i = 0
        t = self.hStart
        self.path_dict_discretized = {t: (x0, y0)}
        if edge[0] in self.delay:
            c = self.delay[edge[0]]//dt
            for j in range(c):
                t += dt
                self.path_dict_discretized[t] = (x0, y0)
        toTravel = dt*drone.speed
        traveled = 0
        
        geometryList = graph.edges[edge]["geometry"]
        if (geometryList[i], geometryList[i+1]) != (graph.nodes[edge[0]]['x'], graph.nodes[edge[0]]['y']):
            geometryList = reverse_geometry_list(geometryList)
        lengthSegment = tools.distance(geometryList[i],geometryList[i+1],geometryList[i+2],geometryList[i+3])
        
        
        while loop: #tools.distance(x0, y0, xEnd, yEnd) > dt*drone.speed:
            if lengthSegment >= toTravel:
                v = (geometryList[i+2]-x0, geometryList[i+3]-y0) 
                x0, y0 = toTravel/lengthSegment*v[0] + x0, toTravel/lengthSegment*v[1] + y0
                t += dt
                self.path_dict_discretized[t] = (x0, y0)
                lengthSegment -= toTravel
                toTravel = dt*drone.speed
                traveled = 0
            else:
                toTravel -= lengthSegment
                traveled += lengthSegment
                try:
                    i += 2
                    lengthSegment = tools.distance(geometryList[i],geometryList[i+1],geometryList[i+2],geometryList[i+3])
                    x0, y0 = geometryList[i], geometryList[i+1]
                except Exception:
                    if edgeIndex < len(self.path)-2:
                        edgeIndex += 1 
                        edge = [self.path[edgeIndex], self.path[edgeIndex+1]]
                        geometryList = graph.edges[edge]["geometry"]
                        i = 0
                        x0, y0 = graph.nodes[edge[0]]['x'], graph.nodes[edge[0]]['y']
                        if (geometryList[i], geometryList[i+1]) != (x0, y0):
                            geometryList = reverse_geometry_list(geometryList.copy())
                        lengthSegment = tools.distance(geometryList[i],geometryList[i+1],geometryList[i+2],geometryList[i+3])
                        if edge[0] in self.delay:
                            c = self.delay[edge[0]]//dt
                            for j in range(c):
                                t += dt
                                self.path_dict_discretized[t] = (x0, y0)
                    else:
                        loop = False
             
        t += traveled/drone.speed
        self.path_dict_discretized[t] = (xEnd, yEnd)


def reverse_geometry_list(listGeo):
    reversedList = []
    for i in range(len(listGeo)-1,0,-2):
        reversedList.append(listGeo[i-1])
        reversedList.append(listGeo[i])
        
    return reversedList


def new_coords(pathDict, coords, lengthSegment, lengthTraveled, t, dt, i, lengthEdge):
    '''Finds the new coordinates for the discretization of the path.'''
    v = (coords[i+2]-coords[i], coords[i+3]-coords[i+1])
    l0 = tools.distance(coords[i], coords[i+1], coords[i+2], coords[i+3])
    x0, y0 = coords[i], coords[i+1]
    offset = lengthTraveled - lengthSegment
    if l0 != 0:
        x, y = offset/l0*v[0] + x0, offset/l0*v[1] + y0

        lengthFromPreviousNode = tools.distance(coords[0], coords[1], x, y)
        
        if lengthFromPreviousNode >= lengthEdge:            
            return False, offset, t, x0, y0
    
        else:
            t += dt
            pathDict[t] = (x, y)
        
            return True, offset, t, x0, y0
    return False, offset, t, x0, y0


###########################
########## Tests ##########
###########################

# dep, arr = '3155094143', '392251'
# path=[dep, '64971266', '29048472', '64975551', '64975746', '64975131', '29048460', '393375', '64975949', '5322689627', '3337813458', '1850015446', '1850015448', '33345291', '251523470', '33345293', '33344817', '33344816', arr]

# dep, arr = '33144706', '33144695'
# path = [dep, '33144601', '33144648', '33144652', '33174087', '33144755', '26405248', '33144463', '64970627', '64972028', '33144491', '33144487', '33144484', '33144459', '33144947', '33144640', '33144591', arr]


# dep, arr = '378699', '33144706'
# path = [dep, '33144414', '3312560802', '33143898', '33144583', '245498398', '33144588', '264055538', '33144591', '33144596', '33144601', arr]

# d = dr.Drone('dr1', dep, arr, 0, 'M600')
# G = md.init_graph(nx.read_graphml("graph_files\\processed_graphM2.graphml"))
# p = Path(G, d, path)
# p.delay = {'64975131': 10}
# d.path = p
# p.discretize_path(5, G, d)
# # print(p.pathDictDiscretized)
# gr.draw_solution(G, d, show_id=False, show_discretized=True, show_time=True, show=False)
# plt.savefig("solutions\\plt_discrete.pdf".format(), dpi=100)


# G = md.init_graph(nx.read_graphml("graph_files\\processed_graphM2.graphml"))
# dep, arr = '64975746', '33345293'
# d = Drone('dr1', dep, arr, 0, 'M600')
# d.add_path([dep, '64975131', '29048460', '393375', '64975949', '5322689627', '3337813458', '1850015446', '1850015448', '33345291', '251523470', arr], G)
# d.path_as_time(5, G)
# edge = d.find_current_edge(15, G)
# print(d.pathDict)
# print(edge)