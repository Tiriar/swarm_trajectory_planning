#include <cstdlib>
#include <ros/ros.h>
#include "UAVController.cpp"

using namespace std;

int main(int argc, char** argv) {
    std::cout << "PROGRAM STARTED" << std::endl;
    ros::init(argc, argv, "boid_controller");
    ros::NodeHandle nh = ros::NodeHandle("~");
    ROS_INFO("Node initialized.");
    /*
    World * world;
    world = new World(nh);

    Controller boid(nh, world);
    // Main loop of the simulation.
    boid.run();

    delete world;
    delete &boid;*/
    ROS_INFO("ENDING THE ROSNODE.");
    std::exit(0);
}

