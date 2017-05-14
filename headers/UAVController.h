#ifndef UAVCONTROLLER_H
#define UAVCONTROLLER_H

#include <cstdio>
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
    ros::NodeHandle nh_;
    std::vector<UAV*> UAVs;
    Eigen::Vector3f startPosition, goalPosition;
    const float RADIUS = 5;
    double measurementStart, measurementEnd;
    bool uavsInCircle(Eigen::Vector3f pos, float r);
    float dist(Eigen::Vector3f v1, Eigen::Vector3f v2);
    float dist2D(Eigen::Vector3f v1, Eigen::Vector3f v2);
    Eigen::Vector3f normalize(Eigen::Vector3f vec);
    std::vector<double> fit(std::vector<double> x, std::vector<double> y);
};


#endif /* UAVCONTROLLER_H */