import math
import matplotlib.pyplot as plt
import numpy
import vrep
from scipy.spatial import Voronoi, voronoi_plot_2d

__author__ = 'BRICH'


def voronoi_diagram(clientID, obstacles):
    """
    2D Voronoi diagram for convex obstacles
    :param clientID: ID of the VRep connection
    :param obstacles: list of obstacle handles
    :return: Voronoi diagram vertices, indices of Voronoi ridge vertices, indices of ridge bounding obstacles
    """

    # number of obstacles
    obsNum = len(obstacles)

    # get obstacle positions
    obsPos = [[0 for _ in range(3)] for _ in range(obsNum)]
    for i in range(obsNum):
        _, obsPos[i] = vrep.simxGetObjectPosition(clientID, obstacles[i], -1, vrep.simx_opmode_oneshot_wait)

    # get bounding points from obstacles
    points = []
    for i in range(len(obsPos)):
        _, minX = vrep.simxGetObjectFloatParameter(clientID, obstacles[i], 15, vrep.simx_opmode_oneshot_wait)
        _, minY = vrep.simxGetObjectFloatParameter(clientID, obstacles[i], 16, vrep.simx_opmode_oneshot_wait)
        _, maxX = vrep.simxGetObjectFloatParameter(clientID, obstacles[i], 18, vrep.simx_opmode_oneshot_wait)
        _, maxY = vrep.simxGetObjectFloatParameter(clientID, obstacles[i], 19, vrep.simx_opmode_oneshot_wait)

        points.append([obsPos[i][0]+maxX, obsPos[i][1]+maxY])
        points.append([obsPos[i][0]+maxX, obsPos[i][1]+minY])
        points.append([obsPos[i][0]+minX, obsPos[i][1]+maxY])
        points.append([obsPos[i][0]+minX, obsPos[i][1]+minY])

    # compute Voronoi diagram
    points = numpy.array(points)
    vor = Voronoi(points)

    # plot result
    voronoi_plot_2d(vor)
    plt.show()

    vert = vor.vertices.tolist()
    ridgeVert = vor.ridge_vertices
    ridgePoints = vor.ridge_points.tolist()

    # delete infinite ridges and ridges inside obstacles
    deleted = 0
    for i in range(len(ridgePoints)):
        if -1 in ridgeVert[i-deleted]:
            del ridgeVert[i-deleted]
            del ridgePoints[i-deleted]
            deleted += 1
        else:
            temp = math.floor(ridgePoints[i-deleted][0]/4)
            if ridgePoints[i-deleted][1] in [0+4*temp, 1+4*temp, 2+4*temp, 3+4*temp]:
                del ridgeVert[i-deleted]
                del ridgePoints[i-deleted]
                deleted += 1

    # delete unconnected vertices
    deleted = 0
    temp = [_ for sublist in ridgeVert for _ in sublist]
    for i in range(len(vert)):
        if i not in temp:
            del vert[i-deleted]
            for j in range(len(ridgeVert)):
                if ridgeVert[j][0] > i-deleted:
                    ridgeVert[j][0] -= 1
                if ridgeVert[j][1] > i-deleted:
                    ridgeVert[j][1] -= 1
            deleted += 1

    # change ridge points to original obstacles
    for pnt in ridgePoints:
        pnt[0] = math.floor(pnt[0]/4)
        pnt[1] = math.floor(pnt[1]/4)

    # plot modified result
    vor.vertices = numpy.array(vert)
    vor.ridge_vertices = ridgeVert
    vor.ridge_points = numpy.array(ridgePoints)
    voronoi_plot_2d(vor)
    plt.show()

    return vert, ridgeVert, ridgePoints
