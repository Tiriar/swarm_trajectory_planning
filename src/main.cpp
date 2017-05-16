#include <cstdlib>
#include <ros/ros.h>
#include "../headers/Checkpoint.h"
#include "../headers/data_fitting.h"

using namespace std;

enum State {MEASURING, FITTING, SEARCHING};
State current = FITTING;

const int FLOCK_SIZE = 2;
const Eigen::Vector3d START = Eigen::Vector3d(5.0, 0.0, 0.0);
const Eigen::Vector3d STOP = Eigen::Vector3d(15.0, 0.0, 0.0);
const float RADIUS = 10;

int main(int argc, char** argv) {
    /*=====MEASURING PART=====*/
    if (current == MEASURING) {
        cout << "===MEASURING PROGRAM STARTED===" << endl;
        ros::init(argc, argv, "swarm_trajectory_planning");
        ros::NodeHandle nh = ros::NodeHandle("~");
        ROS_INFO("NODE INITIALIZED.");

        Checkpoint cp(nh, FLOCK_SIZE);
        cp.measureTime(START, STOP, RADIUS);

        ROS_INFO("ENDING THE ROSNODE.");
    }

    /*=====FITTING PART=====*/
    if (current == FITTING) {
        fitting_run();
    }

    /*=====GRAPH SEARCHING PART=====*/
    if (current == SEARCHING) {
        cout << "NOT YET IMPLEMENTED" << endl;
    }

    exit(0);
}
