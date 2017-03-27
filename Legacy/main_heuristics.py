import boids
import path
import sys
import time
import utils as ut
import vrep
import voronoi_vtk

__author__ = 'BRICH'

# definition of constants
quadsNum = 5                # number of quadrotors
quadsDiam = 0.4             # diameter of quadrotors
speed = 0.2                 # quadrotors speed (path following force in Boids)
obsNum = 19                 # number of obstacles
border = [22000, 22000]     # scene border in millimeters [width, height]
connectStart = 1            # number of vertices to connect to the start vertex
connectEnd = 1              # number of vertices to connect to the end vertex
K = 1                       # number of shortest paths to find
leader = False              # True - leader/followers mode, False - all boids following path

# close all running connections
vrep.simxFinish(-1)

# connect to VRep
print('Trying to connect to remote API...')
clientID = vrep.simxStart("127.0.0.1", 19999, True, True, 2000, 5)

# check if connection was successful
if clientID == -1:
    print('Could not connect to remote API server')
    sys.exit(1)
print('Connected to remote API\n')

# get handles
quads = [0]*quadsNum        # quadrotors handles
targets = [0]*quadsNum      # quadrotors targets handles
sensors = [0]*quadsNum      # proximity sensors handles
obstacles = [0]*obsNum      # obstacles handles

_, quads[0] = vrep.simxGetObjectHandle(clientID, 'Quadricopter', vrep.simx_opmode_oneshot_wait)
_, targets[0] = vrep.simxGetObjectHandle(clientID, 'Quadricopter_target', vrep.simx_opmode_oneshot_wait)
_, sensors[0] = vrep.simxGetObjectHandle(clientID, 'Proximity_sensor', vrep.simx_opmode_oneshot_wait)
for i in range(quadsNum-1):
    _, quads[i+1] = vrep.simxGetObjectHandle(clientID, 'Quadricopter#'+str(i), vrep.simx_opmode_oneshot_wait)
    _, targets[i+1] = vrep.simxGetObjectHandle(clientID, 'Quadricopter_target#'+str(i), vrep.simx_opmode_oneshot_wait)
    _, sensors[i+1] = vrep.simxGetObjectHandle(clientID, 'Proximity_sensor#'+str(i), vrep.simx_opmode_oneshot_wait)

_, obstacles[0] = vrep.simxGetObjectHandle(clientID, 'Obstacle', vrep.simx_opmode_oneshot_wait)
for i in range(obsNum-1):
    _, obstacles[i+1] = vrep.simxGetObjectHandle(clientID, 'Obstacle'+str(i), vrep.simx_opmode_oneshot_wait)

# get quadrotors starting positions
quadsStart = [[0 for _ in range(3)] for _ in range(quadsNum)]
for i in range(quadsNum):
    _, quadsStart[i] = vrep.simxGetObjectPosition(clientID, quads[i], -1, vrep.simx_opmode_oneshot_wait)

# create Voronoi diagram
vert, ridgeVert, ridgeGaps = voronoi_vtk.voro_start(clientID, obstacles, border)

# filter edges with not enough space for quadrotors
i = 0
while i < len(ridgeGaps):
    if ridgeGaps[i] is not None and ridgeGaps[i] < 2*quadsDiam:
        del ridgeVert[i]
        del ridgeGaps[i]
    else:
        i += 1

# filter not connected vertices
count = [0]*len(vert)
for edge in ridgeVert:
    count[edge[0]] += 1
    count[edge[1]] += 1
i = 0
while i < len(vert):
    if count[i] == 0:
        del count[i]
        del vert[i]
        for edge in ridgeVert:
            if edge[0] > i:
                edge[0] -= 1
            if edge[1] > i:
                edge[1] -= 1
    else:
        i += 1

# add Z coordinate to vertices
for point in vert:
    point.append(0.51)

# add start and end vertices
_, temp = vrep.simxGetObjectHandle(clientID, 'Start', vrep.simx_opmode_oneshot_wait)
_, temp = vrep.simxGetObjectPosition(clientID, temp, -1, vrep.simx_opmode_oneshot_wait)
vert.insert(0, temp)
_, temp = vrep.simxGetObjectHandle(clientID, 'End', vrep.simx_opmode_oneshot_wait)
_, temp = vrep.simxGetObjectPosition(clientID, temp, -1, vrep.simx_opmode_oneshot_wait)
vert.append(temp)

# add edges for start and end vertices
for ridge in ridgeVert:
    ridge[0] += 1
    ridge[1] += 1

closest = ut.closest(vert[0], vert, connectStart)
for i in range(connectStart):
    ridgeVert.insert(i, [0, closest[i]])
    ridgeGaps.insert(i, None)

closest = ut.closest(vert[-1], vert, connectEnd)
for i in range(connectEnd):
    ridgeVert.append([len(vert)-1, closest[i]])
    ridgeGaps.append(None)

# evaluate edges and find path
ec = path.heuristic_evaluation(vert, ridgeVert, ridgeGaps, speed, quadsNum)
paths, costs = path.k_shortest(len(vert), ridgeVert, ec, K)

# Voronoi graph visualisation in V-Rep
# black = [0, 0, 0, 0, 0, 0, 64, 64, 64, 0, 0, 0]
# maxCost = max(costs)
# for i in range(1, len(vert)-1):
#     if costs[i] == -1:
#         _, temp = vrep.simxCreateDummy(clientID, 0.1, black, vrep.simx_opmode_oneshot_wait)
#     else:
#         color = ut.get_color(costs[i]/maxCost)+[0, 0, 0, 64, 64, 64, 0, 0, 0]
#         _, temp = vrep.simxCreateDummy(clientID, 0.1, color, vrep.simx_opmode_oneshot_wait)
#     vrep.simxSetObjectPosition(clientID, temp, -1, vert[i], vrep.simx_opmode_oneshot_wait)

# path visualisation in V-Rep
blue = [0, 0, 255, 0, 0, 0, 64, 64, 64, 0, 0, 0]
pathPoints = [0]*len(paths[0])
for i in range(len(paths[0])):
    _, pathPoints[i] = vrep.simxCreateDummy(clientID, 0.15, blue, vrep.simx_opmode_oneshot_wait)

temp = ut.sub(vert[-1], vert[0])
quadsEnd = [[0 for _ in range(3)] for _ in range(quadsNum)]
for i in range(quadsNum):
    quadsEnd[i] = ut.add(quadsStart[i], temp)

for i in range(K):
    print('Current path indices: ' + str(paths[i]))
    foundPath = [[0 for _ in range(3)] for _ in range(len(paths[i]))]
    estimated = 0
    for j in range(len(paths[i])-1):
        v1 = paths[i][j]
        v2 = paths[i][j+1]
        if [v1, v2] in ridgeVert:
            idx = ridgeVert.index([v1, v2])
        else:
            idx = ridgeVert.index([v2, v1])
        estimated += ec[idx]
    print('Estimated operation time: '+str(estimated)+'s\n')
    diff = len(pathPoints)-len(paths[i])
    if diff > 0:
        for j in range(-diff, 0):
            vrep.simxRemoveObject(clientID, pathPoints[j], vrep.simx_opmode_oneshot_wait)
            del pathPoints[j]
    elif diff < 0:
        for j in range(-diff):
            _, temp = vrep.simxCreateDummy(clientID, 0.15, blue, vrep.simx_opmode_oneshot_wait)
            pathPoints.append(temp)
    for j in range(len(paths[i])):
        foundPath[j] = vert[paths[i][j]]
        vrep.simxSetObjectPosition(clientID, pathPoints[j], -1, foundPath[j], vrep.simx_opmode_oneshot_wait)

    # start boids program
    if i % 2 == 1:
        foundPath.reverse()
        for j in range(quadsNum):
            vrep.simxSetObjectPosition(clientID, targets[j], -1, quadsEnd[j], vrep.simx_opmode_oneshot_wait)
        time.sleep(5)
        t = boids.start(clientID, quads, targets, speed, sensors, foundPath, quadsStart, leader)
    else:
        for j in range(quadsNum):
            vrep.simxSetObjectPosition(clientID, targets[j], -1, quadsStart[j], vrep.simx_opmode_oneshot_wait)
        time.sleep(5)
        t = boids.start(clientID, quads, targets, speed, sensors, foundPath, quadsEnd, leader)
    print('Operation time: '+str(t)+'s\n')

# end program and close the connection
print('Simulation end')
vrep.simxFinish(clientID)
