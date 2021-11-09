import Node as nd
import networkx as nx


def Astar(graph:nx.Graph, depId:str, arrId:str):
    currentNode = nd.Node(depId)
    currentNode.cost = 0
    currentNode.heuristic = currentNode.dist_to_node(arrId, graph)
    
    closedList = []
    priorityQueue = [currentNode]

    while len(priorityQueue) > 0:
        currentNode = priorityQueue.pop(0)
        if currentNode.id == arrId:
            return currentNode.path()
        
        for v in graph.adj[currentNode.id]:
            if graph.edges[currentNode.id, v]['open']:
                newCost = currentNode.cost + graph.edges[currentNode.id, v]["length"]
                if v not in closedList:
                    vInPriorityQueue, v = node_in_list(v, priorityQueue)
                    if not vInPriorityQueue:
                        priorityQueue.append(v)
                    if v.cost >= newCost:
                        v.cost = newCost
                        v.heuristic = v.dist_to_node(arrId, graph)
                        v.parent = currentNode
        priorityQueue.sort(key=lambda x: x.f())
        closedList.append(currentNode.id)


def node_in_list(nodeId, nodeList):
    '''Returns wether a node is in a given list and the node object associated to the nodeId'''
    for node in nodeList:
        if node.id == nodeId:
            return True, node
    return False, nd.Node(nodeId)
        
    
    