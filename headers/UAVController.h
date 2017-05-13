#ifndef UAVCONTROLLER_H
#define UAVCONTROLLER_H

#include "../headers/UAV.h"

class UAVController {
public:
    UAVController(const ros::NodeHandle& nh);
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