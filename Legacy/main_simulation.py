import boids
import boids_eval
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
obsNum = 24                 # number of obstacles
border = [22000, 22000]     # scene border in millimeters [width, height]
connectStart = 1            # number of vertices to connect to the start vertex
connectEnd = 1              # number of vertices to connect to the end vertex
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

# use already simulated costs
# with open('data\\dense\\ec5.txt', 'rt', encoding='utf-8') as f:
#     ec = f.readline().split()
# ec = [float(i) for i in ec]

# evaluate edges using heuristics and find path for experimental evaluation
ec = path.heuristic_evaluation(vert, ridgeVert, ridgeGaps, speed, quadsNum)
paths = path.postman(len(vert), ridgeVert, ec)

foundPath = [[0 for _ in range(3)] for _ in range(len(paths))]
for i in range(len(paths)):
    foundPath[i] = vert[paths[i]]

# evaluate edges using boids program
temp = boids_eval.start(clientID, quads, targets, speed, sensors, foundPath, leader)
time.sleep(5)
ec = [sys.maxsize]*len(ec)
for i in range(len(paths)-1):
    p1 = paths[i]
    p2 = paths[i+1]
    if [p1, p2] in ridgeVert:
        idx = ridgeVert.index([p1, p2])
    else:
        idx = ridgeVert.index([p2, p1])
    if ec[idx] > temp[i+1]:
        ec[idx] = temp[i+1]
print()

# find path
paths, costs = path.k_shortest(len(vert), ridgeVert, ec, 1)
print('Found path: '+str(paths[0]))
print('Estimated operation time: '+str(costs[-1])+'s')

foundPath = [[0 for _ in range(3)] for _ in range(len(paths[0]))]
for i in range(len(paths[0])):
    foundPath[i] = vert[paths[0][i]]

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
for i in range(1, len(foundPath)-1):
    _, temp = vrep.simxCreateDummy(clientID, 0.15, blue, vrep.simx_opmode_oneshot_wait)
    vrep.simxSetObjectPosition(clientID, temp, -1, foundPath[i], vrep.simx_opmode_oneshot_wait)

# start boids program
t = boids.start(clientID, quads, targets, speed, sensors, foundPath, None, leader)
print('Operation time: '+str(t)+'s\n')

# end program and close the connection
print('Simulation end')
vrep.simxFinish(clientID)
