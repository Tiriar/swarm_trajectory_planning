#include "../headers/UAVController.h"

/**
 * Constructor for Controller class.
 * - loads the parameters from config file
 * - updated thisUAV variable
 * - starts logging of the odometry
 * - sets publishers
 * @param nh ros NodeHandle
 * @param w Pointer on World, where the UAV will fly
 */
UAVController::UAVController(const ros::NodeHandle& nh, World *w) : world(w), nh_(nh) {
    /*loadParametersFromConfig();

    updateThisUAV();
    getAllOtherUAVs();
    if (logging_enabled) {
        fe = new FileExporter(LOGS_PATH, thisUAV->getName());
    }

    std::ostringstream flyToService, velocityService;
    flyToService << "/";
    flyToService << thisUAV->getName();
    velocityService << flyToService.str();
    flyToService << "/trackers_manager/mpc_tracker/desired_position";
    velocityService << "/velocity";
    flyToPublisher = nh_.advertise<mbzirc_trackers::TrackerPointStamped>(flyToService.str(), 1);
    velocityPublisher = nh_.advertise<nav_msgs::Odometry>(velocityService.str(), 1);
    sim_seq = 0;*/
}

/**
 * Controller destructor.
 */
UAVController::~UAVController() {
    /*flyToPublisher.shutdown();
    velocityPublisher.shutdown();
    delete fe;
    delete thisUAV;*/
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
 * Callback for subscribing of the odometry.
 * @param msg Odometry message containing the position.
 */
void UAVController::positionCallback(const nav_msgs::OdometryConstPtr& msg) {
    //if (ready_to_update) {
    /*position[0] = msg->pose.pose.position.x;
    position[1] = msg->pose.pose.position.y;
    position[2] = msg->pose.pose.position.z;*/
    //}
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
 * Calculate euclidian distance between two vectors.
 * @param v1 First vector.
 * @param v2 Second vector.
 * @return Distance of the two given vectors.
 */
float UAVController::dist(Eigen::Vector3f v1, Eigen::Vector3f v2) {
    Eigen::Vector3f dif = v1 - v2;
    return dif.norm();
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
