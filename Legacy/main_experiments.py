import boids
import sys
import vrep

__author__ = 'Tiriar'

# definition of constants
quadsNum = 5                # number of quadrotors
speed = 0.1                 # quadrotors speed (destination force in Boids)

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
points = [0]*3
path = [[0 for _ in range(3)] for _ in range(len(points))]

_, quads[0] = vrep.simxGetObjectHandle(clientID, 'Quadricopter', vrep.simx_opmode_oneshot_wait)
_, targets[0] = vrep.simxGetObjectHandle(clientID, 'Quadricopter_target', vrep.simx_opmode_oneshot_wait)
_, sensors[0] = vrep.simxGetObjectHandle(clientID, 'Proximity_sensor', vrep.simx_opmode_oneshot_wait)
for i in range(quadsNum-1):
    _, quads[i+1] = vrep.simxGetObjectHandle(clientID, 'Quadricopter#'+str(i), vrep.simx_opmode_oneshot_wait)
    _, targets[i+1] = vrep.simxGetObjectHandle(clientID, 'Quadricopter_target#'+str(i), vrep.simx_opmode_oneshot_wait)
    _, sensors[i+1] = vrep.simxGetObjectHandle(clientID, 'Proximity_sensor#'+str(i), vrep.simx_opmode_oneshot_wait)

_, points[0] = vrep.simxGetObjectHandle(clientID, 'Target', vrep.simx_opmode_oneshot_wait)
_, path[0] = vrep.simxGetObjectPosition(clientID, points[0], -1, vrep.simx_opmode_oneshot_wait)
for i in range(len(points)-1):
    _, points[i+1] = vrep.simxGetObjectHandle(clientID, 'Target'+str(i), vrep.simx_opmode_oneshot_wait)
    _, path[i+1] = vrep.simxGetObjectPosition(clientID, points[i+1], -1, vrep.simx_opmode_oneshot_wait)

boids.start(clientID, quads, targets, speed, sensors, path, None, False)

# end program and close the connection
print('Simulation end')
vrep.simxFinish(clientID)
