#ifndef UAV_H
#define UAV_H

#include <ros/ros.h>
#include <eigen3/Eigen/Core>
#include "string"
#include "sstream"
#include <iostream>
#include <nav_msgs/Odometry.h>

class UAV {
public:
    UAV(const ros::NodeHandle& nh, const std::string &uavName);
    ~UAV();
    Eigen::Vector3f getPosition();
    
private:
    
    void positionCallback(const nav_msgs::OdometryConstPtr& msg);
    Eigen::Vector3f position;
    ros::NodeHandle nh_;
    ros::Subscriber positionSubscriber;
};
#endif /* UAV_H */