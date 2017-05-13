#include "../headers/UAVController.h"
#include <boost/concept_check.hpp>

/**
 * Constructor
 * - sets subscribers
 * @param nh ros NodeHandle
 */
UAVController::UAVController(const ros::NodeHandle& nh, const int &uavCount){

  std::ostringstream uavName;
  for(int i=0;i<uavCount;i++){
    uavName << "/uav" << i;
    UAVs.push_back(UAV(nh,uavName);
  }
}


UAVController::~UAVController() {

}

/**
 * Method to run the controller with. Definitely or indefinitely, based on config variable.
 */
void UAVController::run() {
    // Fill delay queue.
    /*bool runDefinitely = false;
    nh_.param("simulation/run/definitely", runDefinitely, false);
    if (ros::ok() && runDefinitely) {
        int steps = 0;
        nh_.param("simulation/run/number_of_steps", steps, 0);
        for (int i = 0; i < steps; i++) {
            runOneStep();
        }
    } else {
        while (ros::ok()) {
            if (sim_seq++ > 2) {
                runOneStep();
            } else {
                for (UAV * uav : allOtherUAVs) {
                    uav->getPosition();
                }
            }
        }
    }*/
}


/**
 * Run one simulation step. Used in Controller::run() method.
 */
void UAVController::runOneStep() {
    /*Eigen::Vector3f pos = thisUAV->getPosition();
    std::cout << "Step " << sim_seq << ":\n\t- Position: " << pos.transpose();
    Eigen::Vector3f err = Eigen::Vector3f::Zero();
    if (error_localization) {
        err = localizationError(localization_error);
        std::cout << "\n\t\t-With localization error: " << err.transpose() << std::endl;
    }
    pos += err;
    updateCloseUAVs();
    updateCloseBetaAgents();
    thisUAV->acceleration = getFinalForce(pos);
    thisUAV->velocity += thisUAV->acceleration;
    thisUAV->velocity = limit(thisUAV->velocity, max_force);
    if (maintain_constant_altitude) {
        thisUAV->velocity[2] = constant_altitude_value - pos[2];
    } else if (!maintain_constant_altitude && pos[2] < critical_altitude_value) {
        thisUAV->velocity[2] = critical_altitude_value - pos[2];
    }
    pos += thisUAV->velocity;
    publishVelocity();

    std::cout << "\n\t- Flying to: " << pos.transpose() << std::endl;
    flyTo(pos);

    if (logging_enabled) {
        fe->write_odometry(pos, thisUAV->velocity, err);
    }
    ros::Rate(run_rate).sleep();*/
}

/**
 * Calculate euclidean distance between two vectors.
 * @param v1 First vector.
 * @param v2 Second vector.
 * @return Distance of the two given vectors.
 */
float UAVController::dist(Eigen::Vector3f v1, Eigen::Vector3f v2) {
    return (v1-v2).norm();
}

/**
 * Normalize vector. 
 * @param vec Vector to be normalized.
 * @return Normalized vector.
 */
Eigen::Vector3f UAVController::normalize(Eigen::Vector3f vec) {
    float m = vec.norm();
    if (m > 1e-15 && m != 1) {
        vec /= m;
    }
    return vec;
}
