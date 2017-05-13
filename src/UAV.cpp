#include "../headers/UAV.h"

UAV::UAV(const ros::NodeHandle& nh, const std::string &uavName){
  positionSubscriber = nh.subscribe(uavName.str(), 2, &UAV::positionCallback, this);
}

UAV::~UAV() {

}

void UAV::positionCallback(const nav_msgs::OdometryConstPtr& msg) {
    position[0] = msg->pose.pose.position.x;
    position[1] = msg->pose.pose.position.y;
    position[2] = msg->pose.pose.position.z;
}

Eigen::Vector3f UAV::getPosition(){
  return position;
}