#include <cstdlib>
#include <ros/ros.h>
#include "../headers/Checkpoint.h"

using namespace std;
const int FLOCK_SIZE = 2;
const Eigen::Vector3d START = Eigen::Vector3d(5.0, 0.0, 0.0);
const Eigen::Vector3d STOP = Eigen::Vector3d(15.0, 0.0, 0.0);
const float RADIUS = 10;

int main(int argc, char** argv) {
    cout << "===MEASURING PROGRAM STARTED===" << endl;
    ros::init(argc, argv, "swarm_trajectory_planning");
    ros::NodeHandle nh = ros::NodeHandle("~");
    ROS_INFO("NODE INITIALIZED.");

    Checkpoint cp(nh, FLOCK_SIZE);
    cp.measureTime(START, STOP, RADIUS);

    ROS_INFO("ENDING THE ROSNODE.");
    exit(0);
}
