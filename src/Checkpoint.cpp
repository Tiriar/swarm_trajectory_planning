#include "../headers/Checkpoint.h"

using namespace std;

/**
 * Checkpoint constructor - sets subscribers
 * @param nh ros NodeHandle
 * @param uavCount number of UAVs
 */
Checkpoint::Checkpoint(const ros::NodeHandle& nh, const int &uavCount): nh_(nh) {
    for (int i=1; i<uavCount+1; i++) {
        ostringstream uavName;
        uavName << "/uav" << i << "/mbzirc_odom/slow_odom";
        cout << "Creating " << uavName.str().c_str() << endl;
        UAVs.push_back(new UAV(nh_, uavName.str().c_str()));
    }
}

/**
 * Checkpoint destructor - deletes all UAVs
 */
Checkpoint::~Checkpoint() {
    for (UAV *uav : UAVs) {
        delete &uav;
    }
}

/**
 * Measures time from the moment all UAVs enter the starting circle to the moment all UAVs enter the goal circle
 * @param startPos center of the starting circle
 * @param endPos center of the ending circle
 * @param radius radius of the circles
 */
void Checkpoint::measureTime(Eigen::Vector3d startPos, Eigen::Vector3d endPos, float radius) {
    while (ros::ok() && !allIn(startPos, radius)) {
        for (UAV *i : UAVs) {
            Eigen::Vector3d pos = i->getPosition();
            printf("%1.3f, %1.3f, %1.3f\n", pos[0], pos[1], pos[2]);
        }
        ros::Rate(5.0f).sleep();
    }

    double start = ros::Time::now().toSec();
    ROS_INFO("MEASUREMENT START");

    while (ros::ok() && !allIn(endPos, radius)) {
        for (UAV *i :UAVs) {
            Eigen::Vector3d pos = i->getPosition();
            printf("%1.3f, %1.3f, %1.3f\n", pos[0], pos[1], pos[2]);
        }
        ros::Rate(5.0f).sleep();
    }

    double stop = ros::Time::now().toSec();
    ROS_INFO("MEASUREMENT END");
    cout << "FLIGHT DURATION: " << (stop - start) << endl;
}

/**
 * Checks if all UAVs entered a circle
 * @param pos center of the circle
 * @param r radius of the circle
 * @return True if all UAVs entered the circle
 */
bool Checkpoint::allIn(Eigen::Vector3d pos, float r) {
    for (UAV *u : UAVs) {
        if (dist(u->getPosition(), pos) > r) {
            return false;
        }
    }
    return true;
}

/**
 * Calculates 2D euclidean distance between two 3D vectors.
 * @param v1 First vector.
 * @param v2 Second vector.
 * @return Distance of the two given vectors.
 */
double Checkpoint::dist(Eigen::Vector3d v1, Eigen::Vector3d v2) {
    v2[2] = v1[2];
    return (v1-v2).norm();
}
