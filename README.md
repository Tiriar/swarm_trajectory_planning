# swarm_trajectory_planning

Offline trajectory planning for a swarm of UAVs using Boids model. This ROS package is a part of a project developed for the Multi Robot Systems group at the Czech Technical University. The path planning method is based on searching a Voronoi graph created around the obstacles in the environment. The graph's edges are evaluated by a function fitted to experimentally acquired data.

## Prerequisites

### Main MBZIRC repository
This ROS package is based on a solution for MBZIRC challenge developed by the MRS group. You can find the repository of the solution at https://github.com/MBZIRCUPENNCTU/mbzirc. You will need a permission from the MRS group in order to access the repository. Follow their README file in order to install ROS and all the needed packages.

### Boids controller package
Our package can be used as a path planner for any UAV swarm controller solution, but it was developed mainly for the Boids swarm controller. A ROS package implementation of the Boids controller can be found at https://github.com/petrapa6/mbzirc. Simply copy the Boid_controller package into the ROS package folder.

## Usage
The path planning consists of three parts that can be switched between in the config.yaml file in the config folder. The parts are the time measuring part, the evaluation function fitting and the graph search part.

### Time measuring
This part is the only part that needs ROS and a working swarm controller for its function. The path planning uses an evaluation function fitted to the times measured in this part. The experiments are done in a way that multiple times it takes the swarm to fly between two obstacles with different gap sizes between them are measured. These time increments over the time it takes the swarm to fly over the same path without obstacles are used to fit the evaluation function in the next part.

The times were already measured for a Boids controller with fixed parameters and while the data would change with a different settings of the controller, it should not greatly affect the found path. In case you find it necesarry to do new measurements for a different controller, follow these steps:

1) Change the state to 0 and set the constants used for the MEASURING part in the config.yaml file. If you intend to use the given Boids controller, also configure it accordingly.
2) Start up the Gazebo simulator using the gazebo.sh script. You can change the number of quadrotors spawned in the command:
```bash
"Spawn" "sleep 5; spawn 1 2 --enable-bluefox-camera --enable-mobius-camera --enable-rangefinder --run --delete"
```
3) Wait for quadrotors to spawn, then start the scripts uav1.sh, uav2.sh, ..., according to the number of quadrotors spawned. The scripts will start a tmux session, initialize the quadrotors and take off to a given location.
4) Wait for the quadrotors to fly to the given location and stabilize, then start the swarm_trajectory_planning package by the command:
```bash
roslaunch swarm_trajectory_planning simulation.launch
```
5) If you use the given Boids controller, switch to the Boids window in the uav#.sh scripts tmux session and start the prepared command in the window for each quadrotor. If you use your own controller, start it to make the swarm move to the desired location.
6) Wait for the swarm to reach the destination and note the time measured in the terminal you used to start the path planning node.
7) Switch to the GoBack window in the uav#.sh scripts tmux session and start the prepared command in the window for each quadrotor. It will make the quadrotors move back to the initial position.
8) Repeat steps 4 to 7 for different swarm sizes and different gap sizes between obstacles.

### Evaluation function fitting
This part creates the edge evaluation function for the graph search in the final part of the program. A default evaluation function based on our experiments is already prepared and therefore using this part is optional.

If you decided to measure your own times in the Time measuring part, write the times measured to the main.cpp file into the prepared variables in the FITTING part of the program. Change the state to 1 in the config.yaml file, compile and start the program using commands:
```bash
catkin build swarm_trajectory_planning
roslaunch swarm_trajectory_planning simulation.launch
```
The program will compute the evaluation functions and output them on the terminal. Change the evaluate_edge function in the graph.cpp file accordingly.

### Graph search
This is the main part of the path planning program. In order to construct the Voronoi graph used in this part, you need to compile the vtk_voro program provided in \src\vtk_voro. The program takes as an input a text file describing the obstacles in the environment. An example of such a file is provided in the config folder. Start the vtk_voro program by a command:
```bash
./vtk_voro "<path to the .txt file with obstacles>"
```
The vtk_voro program will output a voro_output.txt file. Place this file in the config folder. In the config.yaml file, set the state to 2 and configure the desired parameters for the SEARCHING part. Start the program by a command:
```bash
roslaunch swarm_trajectory_planning simulation.launch
```
The program will output the found path on the terminal.

## Documentation
Doxygen generated documentation is located inside the “documentation” folder. To generate your own version of documentation (after e.g. altering the code’s comments), download the doxygen tool and run the following command in the root of the package repository:
```bash
doxygen doxygen_configuration
```
