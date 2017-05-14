#include <cstdlib>
#include <ros/ros.h>
#include "../headers/UAVController.h"

using namespace std;

int main(int argc, char** argv) {
    std::cout << "SWARM STARTED" << std::endl;
    ros::init(argc, argv, "swarm_trajectory_planning");
    ros::NodeHandle nh = ros::NodeHandle("~");
    ROS_INFO("Node initialized.");

    UAVController controller(nh, 2);
    controller.run();

    delete &controller;
    ROS_INFO("ENDING THE ROSNODE.");
    std::exit(0);
}

