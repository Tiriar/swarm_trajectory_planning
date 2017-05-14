#include "../headers/UAV.h"

using namespace std;

UAV::UAV(const ros::NodeHandle& nh, const char* uavName){
  cout << "UAV initialized for topic " << uavName <<endl;
  name = string(uavName);
  cout << name <<endl;
  this->name = string(uavName);
  cout << name <<endl;
  positionSubscriber = nh_.subscribe(name.c_str(), 1, &UAV::positionCallback, this);
  position = Eigen::Vector3f(-999999.0f,-999999.0f,-999999.0f);
}

UAV::~UAV() {

}

void UAV::positionCallback(const nav_msgs::OdometryConstPtr& msg) {
  cout << "Callback: "<<msg->pose.pose.position.x << ", "<< msg->pose.pose.position.y<< ", "<< msg->pose.pose.position.z<<endl;
  position[0] = msg->pose.pose.position.x;
  position[1] = msg->pose.pose.position.y;
  position[2] = msg->pose.pose.position.z;
}

Eigen::Vector3f UAV::getPosition(){
  updatePosition();
  return position;
}

void UAV::updatePosition() {
    ros::spinOnce();
    ros::Duration(0.05).sleep();
}