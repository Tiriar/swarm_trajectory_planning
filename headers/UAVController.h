#ifndef UAVCONTROLLER_H
#define UAVCONTROLLER_H

#include <mbzirc_trackers/TrackerPointStamped.h>

class UAVController {
public:
    UAVController(const ros::NodeHandle& nh);
    ~UAVController();
    void run();

private:
    void runOneStep();
    void positionCallback(const nav_msgs::OdometryConstPtr& msg);
    
    // Defining methods and so on
    ros::NodeHandle nh_;
    
    std::vector<std::string> UAVs;

    ros::Subscriber positionSubscriber;
    float dist(Eigen::Vector3f v1, Eigen::Vector3f v2);
};


#endif /* UAVCONTROLLER_H */