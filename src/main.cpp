#include <cstdlib>
#include <ros/ros.h>
#include "../headers/Checkpoint.h"
#include "../headers/data_fitting.h"
#include "../headers/graph.h"

using namespace std;

enum State {MEASURING, FITTING, SEARCHING};
State current = SEARCHING;

const int FLOCK_SIZE = 2;
const Eigen::Vector3d START = Eigen::Vector3d(5.0, 0.0, 0.0);
const Eigen::Vector3d STOP = Eigen::Vector3d(15.0, 0.0, 0.0);
const float RADIUS = 10;

int main(int argc, char** argv) {
    ros::init(argc, argv, "swarm_trajectory_planning");
    ros::NodeHandle nh = ros::NodeHandle("~");
    ROS_INFO("NODE INITIALIZED.");

    /*=====MEASURING PART=====*/
    if (current == MEASURING) {
        Checkpoint cp(nh, FLOCK_SIZE);
        cp.measureTime(START, STOP, RADIUS);
    }

    /*=====FITTING PART=====*/
    if (current == FITTING) {
        fitting_run();
    }

    /*=====GRAPH SEARCHING PART=====*/
    if (current == SEARCHING) {
        string voro_path;
        nh.getParam("voro_path", voro_path);
        vector<vector<double>> vertices = load_vertices(voro_path);
        vector<vector<int>> edges = load_edges(voro_path);
    }

    ROS_INFO("ENDING THE ROSNODE.");
    exit(0);
}
