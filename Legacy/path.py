import math
import sys
import utils as ut

__author__ = 'BRICH'


def heuristic_evaluation(points, edges, gaps, speed, quadNum):
    """
    Evaluates the edges based on experimental heuristics
    :param points: list of point coordinates
    :param edges: edges between points
    :param gaps: gaps between obstacles for each edge
    :param speed: max speed of quadrotors
    :param quadNum: number of quadrotors
    :return: evaluated edges (edge costs)
    """
    costs = [0]*len(edges)
    for i in range(len(edges)):
        length = ut.norm(ut.sub(points[edges[i][0]], points[edges[i][1]]))
        costs[i] = 0.117*length/speed

        if gaps[i] is not None:
            if quadNum == 1:
                costs[i] += 3.87*math.exp(-0.77*gaps[i])
            else:
                costs[i] += (4.353+2.957*quadNum)*math.exp(-(1.215+0.004*quadNum)*gaps[i])
    return costs


def k_shortest(pntnum, edges, ec, K):
    """
    Finds K paths using K shortest paths routing algorithm
    :param pntnum: number of graph points
    :param edges: edges between points
    :param ec: cost of each edge
    :param K: number of paths to find by K shortest paths algorithm
    :return: list of paths by indices, costs for each discovered node
    """
    s = 0
    t = pntnum-1
    P = []
    B = [[0, [s]]]
    count = [0]*pntnum
    costs = [-1]*pntnum
    idx = 0
    while len(B) != 0 and count[t] <= K:
        C = B[0][0]
        Pu = B[0][1]
        u = Pu[-1]
        del B[0]
        count[u] += 1
        if u == t:
            P.append(Pu)
        if count[u] <= K:
            for v in range(pntnum):
                found = False
                if [v, u] in edges:
                    found = True
                    idx = edges.index([v, u])
                elif [u, v] in edges:
                    found = True
                    idx = edges.index([u, v])
                if found and v not in Pu:
                    Pv = Pu.copy()
                    Pv.append(v)
                    C += ec[idx]
                    if costs[v] == -1 or C < costs[v]:
                        costs[v] = C
                    idx = 0
                    while len(B) > idx and B[idx][0] < C:
                        idx += 1
                    B.insert(idx, [C, Pv])
    return P, costs


def postman(pntnum, edges, ec):
    """
    Finds a path in graph that covers all edges (not optimal solution)
    :param pntnum: number of graph points
    :param edges: edges between points
    :param ec: cost of each edge
    :return: path by point indices
    """
    edgenum = len(edges)
    visited = []
    path = [0]
    while len(visited) != edgenum:
        shortest = None
        cost = sys.maxsize
        for i in range(edgenum):
            if path[-1] in edges[i] and edges[i] not in visited and ec[i] < cost:
                shortest = edges[i]
                cost = ec[i]
        if shortest is None:
            temp = closest_unvisited(pntnum, edges, ec, path[-1], visited)
            for i in range(1, len(temp)):
                path.append(temp[i])
        else:
            if path[-1] == shortest[0]:
                new = shortest[1]
            else:
                new = shortest[0]
            path.append(new)
            visited.append(shortest)
    return path


def closest_unvisited(pntnum, edges, ec, start, visited):
    """
    Finds a path in a graph from the starting point to the closest unvisited edge
    :param pntnum: number of graph points
    :param edges: edges between points
    :param ec: cost of each edge
    :param start: starting point
    :param visited: list of visited edges
    :return: path by point indices
    """
    terminals = []
    for edge in edges:
        if edge not in visited:
            if edge[0] not in terminals:
                terminals.append(edge[0])
            if edge[1] not in terminals:
                terminals.append(edge[1])

    B = [[0, [start]]]
    count = [0]*pntnum
    idx = 0
    while len(B) != 0:
        C = B[0][0]
        Pu = B[0][1]
        u = Pu[-1]
        del B[0]
        count[u] += 1
        if u in terminals:
            return Pu
        if count[u] <= 1:
            for v in range(pntnum):
                found = False
                if [v, u] in edges:
                    found = True
                    idx = edges.index([v, u])
                elif [u, v] in edges:
                    found = True
                    idx = edges.index([u, v])
                if found and v not in Pu:
                    Pv = Pu.copy()
                    Pv.append(v)
                    C += ec[idx]
                    idx = 0
                    while len(B) > idx and B[idx][0] < C:
                        idx += 1
                    B.insert(idx, [C, Pv])
    return None
