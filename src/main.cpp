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

    int state, flock_size;
    nh.param("state", state, 0);
    nh.param("flock_size", flock_size, 2);

    /*=====MEASURING PART=====*/
    if (state == 0) {
        double x, y, z;
        float radius;
        nh.param("start/x", x, 0.0);
        nh.param("start/y", y, 0.0);
        nh.param("start/z", z, 0.0);
        Eigen::Vector3d start = Eigen::Vector3d(x, y, z);
        nh.param("stop/x", x, 0.0);
        nh.param("stop/y", y, 0.0);
        nh.param("stop/z", z, 0.0);
        Eigen::Vector3d stop = Eigen::Vector3d(x, y, z);
        nh.param("radius", radius, 0.0f);

        Checkpoint cp(nh, flock_size);
        cp.measureTime(start, stop, radius);
    }

    /*=====FITTING PART=====*/
    else if (state == 1) {
        double time_without_obstacles = 1.584; // normalized to 1 meter!
        // x[i] = gap size for i-th measurement
        vector<double> x = {1.0, 1.2, 1.4, 1.6, 1.8, 2.0, 2.2, 2.4, 2.6, 2.8, 3.0, 3.5, 4.0};
        // y[FLOCK_SIZE][i] = measured time increment for i-th gap size (normalized to 1 meter!)
        vector<vector<double>> y = {{5.700, 4.796, 4.200, 3.998, 3.602, 3.298, 3.008, 2.896, 2.685, 2.433, 2.381, 2.022, 1.809},
                                    {7.314, 6.698, 6.138, 5.752, 5.298, 4.998, 4.314, 4.092, 3.800, 3.396, 3.094, 2.468, 1.998},
                                    {9.515, 8.836, 7.951, 7.536, 7.048, 6.521, 6.093, 5.684, 5.281, 4.935, 4.729, 4.112, 3.598}};
        fitting_run(time_without_obstacles, x, y);
    }

    /*=====GRAPH SEARCHING PART=====*/
    else if (state == 2) {
        int source_idx, goal_idx;
        string voro_path;
        nh.param("start_node", source_idx, 0);
        nh.param("end_node", goal_idx, 0);
        nh.getParam("voro_path", voro_path);

        vector<int> path = find_path(voro_path, flock_size, source_idx, goal_idx);
        cout << "Found path:";
        for (int i=0; i<path.size(); i++) {
            cout << " " << path[i];
        }
        cout << endl;
    }

    ROS_INFO("ENDING THE ROSNODE.");
    exit(0);
}
