#ifndef UAVCONTROLLER_H
#define UAVCONTROLLER_H


#include <ros/ros.h>
#include <eigen3/Eigen/Core>
#include "string"
#include "sstream"
#include <iostream>
#include <nav_msgs/Odometry.h>
#include <cmath>

#include "../headers/UAV.h"

class UAVController {
public:
    UAVController(const ros::NodeHandle& nh, const int &uavCount);
    ~UAVController();
    void run();

private:
    void runOneStep();
    
    // Defining methods and so on
    ros::NodeHandle nh_;
    
    std::vector<UAV> UAVs;

    float dist(Eigen::Vector3f v1, Eigen::Vector3f v2);
};


#endif /* UAVCONTROLLER_H */