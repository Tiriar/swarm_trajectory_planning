#ifndef CHECKPOINT_H
#define CHECKPOINT_H

#include <cstdio>
#include <ros/ros.h>
#include <eigen3/Eigen/Core>
#include "string"
#include "sstream"
#include <iostream>
#include <nav_msgs/Odometry.h>
#include "../headers/UAV.h"

class Checkpoint {
public:
    Checkpoint(const ros::NodeHandle& nh, const int &uavCount);
    ~Checkpoint();
    void measureTime(Eigen::Vector3d startPos, Eigen::Vector3d endPos, float radius);
private:
    ros::NodeHandle nh_;
    std::vector<UAV*> UAVs;

    bool allIn(Eigen::Vector3d pos, float r);
    double dist(Eigen::Vector3d v1, Eigen::Vector3d v2);
};

#endif /* CHECKPOINT_H */
