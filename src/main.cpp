#include <cstdlib>
#include <ros/ros.h>
#include "../headers/Checkpoint.h"
#include "../headers/data_fitting.h"
#include "../headers/graph.h"

using namespace std;

int main(int argc, char** argv) {
    ros::init(argc, argv, "swarm_trajectory_planning");
    ros::NodeHandle nh = ros::NodeHandle("~");
    ROS_INFO("NODE INITIALIZED.");

    int state;
    nh.param("state", state, 0);

    /*=====MEASURING PART=====*/
    if (state == 0) {
        int flock_size;
        nh.param("flock_size", flock_size, 2);

        double x, y, z;
        nh.param("start/x", x, 0.0);
        nh.param("start/y", y, 0.0);
        nh.param("start/z", z, 0.0);
        Eigen::Vector3d start = Eigen::Vector3d(x, y, z);
        nh.param("stop/x", x, 0.0);
        nh.param("stop/y", y, 0.0);
        nh.param("stop/z", z, 0.0);
        Eigen::Vector3d stop = Eigen::Vector3d(x, y, z);

        float radius;
        nh.param("radius", radius, 0.0f);

        Checkpoint cp(nh, flock_size);
        cp.measureTime(start, stop, radius);
    }

    /*=====FITTING PART=====*/
    else if (state == 1) {
        fitting_run();
    }

    /*=====GRAPH SEARCHING PART=====*/
    else if (state == 2) {
        string voro_path;
        nh.getParam("voro_path", voro_path);
        vector<vector<double>> vertices = load_vertices(voro_path);
        vector<vector<int>> edges = load_edges(voro_path);
    }

    ROS_INFO("ENDING THE ROSNODE.");
    exit(0);
}
