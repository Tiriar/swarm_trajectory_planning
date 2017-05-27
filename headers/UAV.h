#ifndef UAV_H
#define UAV_H

#include <cstdio>
#include <ros/ros.h>
#include <eigen3/Eigen/Core>
#include "string"
#include "sstream"
#include <iostream>
#include <nav_msgs/Odometry.h>

class UAV {
public:
    UAV(const ros::NodeHandle& nh, const char* uavName);
    ~UAV();
    Eigen::Vector3d getPosition();
private:
  
  /** UAV name in Gazebo*/
    std::string name;
    
  /** Latest UAV position*/
    Eigen::Vector3d position;
    
  /** ROS node handle*/
    ros::NodeHandle nh_;
    
  /** ROS subscriber for reading the UAV position*/
    ros::Subscriber positionSubscriber;

    void updatePosition();
    void positionCallback(const nav_msgs::OdometryConstPtr& msg);
};

#endif /* UAV_H */
