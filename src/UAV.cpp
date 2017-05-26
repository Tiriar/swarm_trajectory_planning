#include "../headers/UAV.h"

using namespace std;

/**
 * UAV constructor
 * @param nh ros NodeHandle
 * @param uavName name of the UAV
 */
UAV::UAV(const ros::NodeHandle& nh, const char* uavName) {
    cout << "UAV initialized for topic " << uavName << endl;
    name = string(uavName);
    positionSubscriber = nh_.subscribe(name.c_str(), 1, &UAV::positionCallback, this);
    position = Eigen::Vector3d(-9999.0, -9999.0, -9999.0);
}

/**
 * UAV destructor
 */
UAV::~UAV() {
    cout << "UAV " << name << " deleted" << endl;
}

/**
 * UAV position callback
 * @param msg odometry message
 */
void UAV::positionCallback(const nav_msgs::OdometryConstPtr& msg) {
    position[0] = msg->pose.pose.position.x;
    position[1] = msg->pose.pose.position.y;
    position[2] = msg->pose.pose.position.z;
}

/**
 * UAV position getter
 * @return UAV position
 */
Eigen::Vector3d UAV::getPosition() {
    updatePosition();
    return position;
}

/**
 * Updates position of the UAV
 */
void UAV::updatePosition() {
    ros::spinOnce();
    ros::Duration(0.05).sleep();
}
