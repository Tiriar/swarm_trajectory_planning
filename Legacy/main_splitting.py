import boids_splitting
import path
import sys
import utils as ut
import vrep
import voronoi_vtk

__author__ = 'BRICH'

# definition of constants
quadsNum = 5                # number of quadrotors
quadsDiam = 0.4             # diameter of quadrotors
speed = 0.2                 # quadrotors speed (path following force in Boids)
connectStart = 1            # number of vertices to connect to the start vertex
connectEnd = 1              # number of vertices to connect to the end vertex

# close all running connections
vrep.simxFinish(-1)

# connect to VRep
print('Trying to connect to remote API...')
clientID = vrep.simxStart("127.0.0.1", 19999, True, True, 2000, 5)

# get handles
quads = [0]*quadsNum        # quadrotors handles
targets = [0]*quadsNum      # quadrotors targets handles
sensors = [0]*quadsNum      # proximity sensors handles

_, quads[0] = vrep.simxGetObjectHandle(clientID, 'Quadricopter', vrep.simx_opmode_oneshot_wait)
_, targets[0] = vrep.simxGetObjectHandle(clientID, 'Quadricopter_target', vrep.simx_opmode_oneshot_wait)
_, sensors[0] = vrep.simxGetObjectHandle(clientID, 'Proximity_sensor', vrep.simx_opmode_oneshot_wait)
for i in range(quadsNum-1):
    _, quads[i+1] = vrep.simxGetObjectHandle(clientID, 'Quadricopter#'+str(i), vrep.simx_opmode_oneshot_wait)
    _, targets[i+1] = vrep.simxGetObjectHandle(clientID, 'Quadricopter_target#'+str(i), vrep.simx_opmode_oneshot_wait)
    _, sensors[i+1] = vrep.simxGetObjectHandle(clientID, 'Proximity_sensor#'+str(i), vrep.simx_opmode_oneshot_wait)

# check if connection was successful
if clientID == -1:
    print('Could not connect to remote API server')
    sys.exit(1)
print('Connected to remote API\n')

# load map and create Voronoi diagram
vertices, edges, gaps = voronoi_vtk.load_map([0, 0], 1, True, clientID)

# filter edges with not enough space for quadrotors
i = 0
while i < len(gaps):
    if gaps[i] is not None and gaps[i] < 2*quadsDiam:
        del edges[i]
        del gaps[i]
    else:
        i += 1

# filter not connected vertices
count = [0]*len(vertices)
for edge in edges:
    count[edge[0]] += 1
    count[edge[1]] += 1
i = 0
while i < len(vertices):
    if count[i] == 0:
        del count[i]
        del vertices[i]
        for edge in edges:
            if edge[0] > i:
                edge[0] -= 1
            if edge[1] > i:
                edge[1] -= 1
    else:
        i += 1

# add Z coordinate to vertices
for point in vertices:
    point.append(0.51)

# show vertices in V-Rep
# for vertex in vertices:
#     _, tmp = vrep.simxCreateDummy(clientID, 0.1, None, vrep.simx_opmode_oneshot_wait)
#     vrep.simxSetObjectPosition(clientID, tmp, -1, vertex, vrep.simx_opmode_oneshot_wait)

# add start and end vertices
_, temp = vrep.simxGetObjectHandle(clientID, 'Start', vrep.simx_opmode_oneshot_wait)
_, temp = vrep.simxGetObjectPosition(clientID, temp, -1, vrep.simx_opmode_oneshot_wait)
vertices.insert(0, temp)
_, temp = vrep.simxGetObjectHandle(clientID, 'End', vrep.simx_opmode_oneshot_wait)
_, temp = vrep.simxGetObjectPosition(clientID, temp, -1, vrep.simx_opmode_oneshot_wait)
vertices.append(temp)

# add edges for start and end vertices
for ridge in edges:
    ridge[0] += 1
    ridge[1] += 1

closest = ut.closest(vertices[0], vertices, connectStart)
for i in range(connectStart):
    edges.insert(i, [0, closest[i]])
    gaps.insert(i, None)

closest = ut.closest(vertices[-1], vertices, connectEnd)
for i in range(connectEnd):
    edges.append([len(vertices)-1, closest[i]])
    gaps.append(None)

# load the paths
paths = []
with open('data\\splitting\\maze_heuristika-15-200.txt', 'rt', encoding='utf-8') as f:
    for i in range(quadsNum):
        tmp = f.readline().split()
        tmp = [int(j) for j in tmp]
        paths.append(tmp)

# load the evaluation
ec = [path.heuristic_evaluation(vertices, edges, gaps, speed, 1),
      path.heuristic_evaluation(vertices, edges, gaps, speed, 2),
      path.heuristic_evaluation(vertices, edges, gaps, speed, 3),
      path.heuristic_evaluation(vertices, edges, gaps, speed, 4),
      path.heuristic_evaluation(vertices, edges, gaps, speed, 5)]
# with open('data\\maze\\ec1.txt', 'rt', encoding='utf-8') as f:
#     ec1 = f.readline().split()
# ec1 = [float(i) for i in ec1]
# with open('data\\maze\\ec2.txt', 'rt', encoding='utf-8') as f:
#     ec2 = f.readline().split()
# ec2 = [float(i) for i in ec2]
# with open('data\\maze\\ec3.txt', 'rt', encoding='utf-8') as f:
#     ec3 = f.readline().split()
# ec3 = [float(i) for i in ec3]
# with open('data\\maze\\ec4.txt', 'rt', encoding='utf-8') as f:
#     ec4 = f.readline().split()
# ec4 = [float(i) for i in ec4]
# with open('data\\maze\\ec5.txt', 'rt', encoding='utf-8') as f:
#     ec5 = f.readline().split()
# ec5 = [float(i) for i in ec5]
# ec = [ec1, ec2, ec3, ec4, ec5]

# compute estimated time
times = [0]*quadsNum
for i in range(quadsNum):
    for j in range(len(paths[i])-1):
        v1 = paths[i][j]
        v2 = paths[i][j+1]
        ecnum = 0
        for k in range(quadsNum):
            if k != i and v1 in paths[k]:
                idx = paths[k].index(v1)
                if paths[k][idx+1] == v2:
                    ecnum += 1
        if [v1, v2] in edges:
            idx = edges.index([v1, v2])
        else:
            idx = edges.index([v2, v1])
        times[i] += ec[ecnum][idx]
print('Estimated times:')
for i in range(quadsNum):
    print('Quad #'+str(i)+': '+str(times[i])+'s')
print()

# start the boids program
t = boids_splitting.start(clientID, quads, targets, speed, sensors, vertices, paths)
