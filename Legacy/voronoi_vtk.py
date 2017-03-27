import math
import os
import sys
import utils as ut
import vrep

__author__ = 'BRICH'


def voro_start(clientID, obstacles, border):
    """
    Creates a Voronoi diagram around obstacles using vtk_voro
    :param clientID: ID of the VRep connection
    :param obstacles: list of obstacle handles
    :param border: scene border [width, height]
    :return: list of graph vertices, indices of edge vertices, gaps for each edge
    """

    border = [[-border[0]/2, -border[1]/2],
              [-border[0]/2, border[1]/2],
              [border[0]/2, border[1]/2],
              [border[0]/2, -border[1]/2]]
    obsNum = len(obstacles)
    output = []
    for i in range(obsNum):
        # get data from V-Rep
        _, position = vrep.simxGetObjectPosition(clientID, obstacles[i], -1, vrep.simx_opmode_oneshot_wait)
        _, orientation = vrep.simxGetObjectOrientation(clientID, obstacles[i], -1, vrep.simx_opmode_oneshot_wait)
        orientation = orientation[2]

        _, minX = vrep.simxGetObjectFloatParameter(clientID, obstacles[i], 15, vrep.simx_opmode_oneshot_wait)
        _, minY = vrep.simxGetObjectFloatParameter(clientID, obstacles[i], 16, vrep.simx_opmode_oneshot_wait)
        _, maxX = vrep.simxGetObjectFloatParameter(clientID, obstacles[i], 18, vrep.simx_opmode_oneshot_wait)
        _, maxY = vrep.simxGetObjectFloatParameter(clientID, obstacles[i], 19, vrep.simx_opmode_oneshot_wait)

        corners = [[minX, minY], [minX, maxY], [maxX, minY], [maxX, maxY]]

        # compute bounding box corners for angled object
        center = ut.mul(ut.add(corners[0], corners[3]), 0.5)
        radius = ut.norm(ut.sub(corners[0], center))
        zero = ut.mul(ut.add(corners[0], corners[2]), 0.5)

        angle = ut.angle(ut.sub(corners[0], center), ut.sub(zero, center))+orientation+math.pi/2
        corners[0][0] = center[0]+radius*math.cos(angle)
        corners[0][1] = center[1]+radius*math.sin(angle)

        angle = ut.angle(ut.sub(corners[1], center), ut.sub(zero, center))+orientation+math.pi/2
        corners[1][0] = center[0]+radius*math.cos(angle)
        corners[1][1] = center[1]+radius*math.sin(angle)

        vec = ut.sub(center, corners[0])
        corners[2] = ut.add(center, vec)

        vec = ut.sub(center, corners[1])
        corners[3] = ut.add(center, vec)

        # move to obstacle position and convert to millimeters
        for j in range(4):
            corners[j] = ut.add(position[0:2], corners[j])
            for k in range(2):
                corners[j][k] = int(corners[j][k]*1000)
        output.append(corners)

    # merge overlapping obstacles
    ignore = []
    merged = []
    for i in range(obsNum):
        if i not in ignore:
            tmp, visited = ut.merge_obstacles(i, output, ignore)
            ignore = ignore+visited
            for j in range(len(tmp)):
                for k in range(2):
                    tmp[j][k] = int(tmp[j][k])
            merged.append(tmp)

    # write to file to be used by vtk_voro
    file = open('voro_data.txt', 'wt', encoding='utf-8')
    file.write('[BORDER]\n' +
               str(border[0][0]) + ' ' + str(border[0][1]) + '\n' +
               str(border[1][0]) + ' ' + str(border[1][1]) + '\n' +
               str(border[2][0]) + ' ' + str(border[2][1]) + '\n' +
               str(border[3][0]) + ' ' + str(border[3][1]) + '\n\n')
    for i in range(len(merged)):
        file.write('[OBSTACLE]\n')
        for j in range(len(merged[i])):
            file.write(str(merged[i][j][0]) + ' ' + str(merged[i][j][1]) + '\n')
        file.write('\n')
    file.close()

    # execute vtk_voro
    command = "start /wait cmd /c cd " + os.getcwd() + " && .\\vtk_voro\\build\\Debug\\vtk_voro.exe voro_data.txt"
    os.system(command)

    # read vtk_voro output
    vertices = []
    edges = []
    mode = True
    with open('voro_output.txt', 'rt', encoding='utf-8') as f:
        for line in f:
            tmp = line.split()
            if tmp[0] == "---":
                mode = False
                continue
            if mode:
                for i in range(len(tmp)):
                    tmp[i] = float(tmp[i])/1000
                vertices.append(tmp)
            else:
                for i in range(len(tmp)):
                    tmp[i] = int(tmp[i])
                edges.append(tmp)

    # sample the obstacles for computing gaps
    merged.append(border)
    for i in range(len(merged)):
        for j in range(len(merged[i])):
            for k in range(len(merged[i][j])):
                merged[i][j][k] /= 1000

    for i in range(len(merged)):
        size = len(merged[i])
        for j in range(size):
            if j == size-1:
                p0 = merged[i][j]
                p1 = merged[i][0]
            else:
                p0 = merged[i][j]
                p1 = merged[i][j+1]
            vec = ut.sub(p1, p0)
            length = ut.norm(vec)
            vec = ut.mul(vec, 1/length)

            for k in range(int(length/0.1)):
                merged[i].append(ut.add(p0, ut.mul(vec, (k+1)*0.1)))

    # find size of the gap for each edge
    gaps = []
    for edge in edges:
        p0 = vertices[edge[0]]
        p1 = vertices[edge[1]]
        center = ut.mul(ut.add(p0, p1), 0.5)
        vec = ut.sub(p1, p0)
        norm = [-vec[1], vec[0]]
        length = ut.norm(vec)
        line = [norm[0], norm[1], -norm[0]*p0[0]-norm[1]*p0[1]]
        normal = [vec[0], vec[1], -vec[0]*center[0]-vec[1]*center[1]]

        mindist = [sys.maxsize, sys.maxsize]
        closest = [None, None]
        for obstacle in merged:
            for point in obstacle:
                if ut.point_line(point, normal) <= length/2+0.01:
                    dist = ut.point_line(point, line)
                    if line[0]*point[0]+line[1]*point[1]+line[2] > 0:
                        if dist < mindist[0]:
                            mindist[0] = dist
                            closest[0] = point
                    elif dist < mindist[1]:
                        mindist[1] = dist
                        closest[1] = point
        if None in closest:
            gaps.append(None)
        else:
            gaps.append(ut.norm(ut.sub(closest[0], closest[1])))

    return vertices, edges, gaps


def load_map(shift, enlarge, include_border, clientID):
    """
    Loads map from voro_data.txt, creates Voronoi diagram and adjusts the output
    :param shift: shift the map by [x,y]
    :param enlarge: enlarge the map
    :param include_border: include border when computing size of gaps
    :param clientID: ID of the VRep connection (only for visualization)
    :return: list of graph vertices, indices of edge vertices, gaps for each edge
    """

    # read map file
    border = []
    obstacles = [[]]
    mode = True
    idx = 0
    with open('voro_data.txt', 'rt', encoding='utf-8') as f:
        for line in f:
            tmp = line.split()
            if tmp:
                if tmp[0].lower() == '[border]':
                        mode = True
                elif tmp[0].lower() == '[obstacle]':
                    obstacles.append([])
                    idx += 1
                    mode = False
                else:
                    if mode:
                        tmp[0] = float(tmp[0])+shift[0]
                        tmp[1] = float(tmp[1])+shift[1]
                        for i in range(len(tmp)):
                            tmp[i] = enlarge*float(tmp[i])/1000
                        border.append(tmp)
                    else:
                        tmp[0] = float(tmp[0])+shift[0]
                        tmp[1] = float(tmp[1])+shift[1]
                        for i in range(len(tmp)):
                            tmp[i] = enlarge*float(tmp[i])/1000
                        obstacles[idx].append(tmp)
    del obstacles[0]

    # execute vtk_voro
    command = "start /wait cmd /c cd " + os.getcwd() + " && .\\vtk_voro\\build\\Debug\\vtk_voro.exe voro_data.txt"
    os.system(command)

    # read vtk_voro output
    vertices = []
    edges = []
    mode = True
    with open('voro_output.txt', 'rt', encoding='utf-8') as f:
        for line in f:
            tmp = line.split()
            if tmp[0] == "---":
                mode = False
                continue
            if mode:
                tmp[0] = float(tmp[0])+shift[0]
                tmp[1] = float(tmp[1])+shift[1]
                for i in range(len(tmp)):
                    tmp[i] = enlarge*float(tmp[i])/1000
                vertices.append(tmp)
            else:
                for i in range(len(tmp)):
                    tmp[i] = int(tmp[i])
                edges.append(tmp)

    # for vertex in vertices:
    #     _, tmp = vrep.simxCreateDummy(clientID, 0.1, None, vrep.simx_opmode_oneshot_wait)
    #     vrep.simxSetObjectPosition(clientID, tmp, -1, vertex, vrep.simx_opmode_oneshot_wait)

    # sample the obstacles for computing gaps
    if include_border:
        obstacles.append(border)

    # for obstacle in obstacles:
    #     for point in obstacle:
    #         _, tmp = vrep.simxCreateDummy(clientID, 0.1, None, vrep.simx_opmode_oneshot_wait)
    #         vrep.simxSetObjectPosition(clientID, tmp, -1, point, vrep.simx_opmode_oneshot_wait)

    for i in range(len(obstacles)):
        size = len(obstacles[i])
        for j in range(size):
            if j == size-1:
                p0 = obstacles[i][j]
                p1 = obstacles[i][0]
            else:
                p0 = obstacles[i][j]
                p1 = obstacles[i][j+1]
            vec = ut.sub(p1, p0)
            length = ut.norm(vec)
            vec = ut.mul(vec, 1/length)

            for k in range(int(length/0.1)):
                obstacles[i].append(ut.add(p0, ut.mul(vec, (k+1)*0.1)))

    # for obstacle in obstacles:
    #     for point in obstacle:
    #         _, tmp = vrep.simxCreateDummy(clientID, 0.1, None, vrep.simx_opmode_oneshot_wait)
    #         vrep.simxSetObjectPosition(clientID, tmp, -1, point, vrep.simx_opmode_oneshot_wait)

    # find size of the gap for each edge
    gaps = []
    for edge in edges:
        p0 = vertices[edge[0]]
        p1 = vertices[edge[1]]
        center = ut.mul(ut.add(p0, p1), 0.5)
        vec = ut.sub(p1, p0)
        norm = [-vec[1], vec[0]]
        length = ut.norm(vec)
        line = [norm[0], norm[1], -norm[0]*p0[0]-norm[1]*p0[1]]
        normal = [vec[0], vec[1], -vec[0]*center[0]-vec[1]*center[1]]

        mindist = [sys.maxsize, sys.maxsize]
        found = [None, None]
        for obstacle in obstacles:
            for point in obstacle:
                if ut.point_line(point, normal) <= length/2+0.01:
                    dist = ut.point_line(point, line)
                    if line[0]*point[0]+line[1]*point[1]+line[2] > 0:
                        if dist < mindist[0]:
                            mindist[0] = dist
                            found[0] = True
                    elif dist < mindist[1]:
                        mindist[1] = dist
                        found[1] = True
        if None in found:
            gaps.append(None)
        else:
            gaps.append(sum(mindist))

    return vertices, edges, gaps
