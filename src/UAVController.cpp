#include "../headers/UAVController.h"

/**
 * Constructor
 * - sets subscribers
 * @param nh ros NodeHandle
 */
UAVController::UAVController(const ros::NodeHandle& nh, const int &uavCount){
  std::ostringstream uavName;
  for(int i=0;i<uavCount;i++){
    uavName << "/uav" << i;
//     UAVs.push_back(UAV(nh,uavName.str()));
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
    } else {*/
    while (ros::ok()) {
      runOneStep();
    }
           /* } else {
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
    }*/
    for(UAV i :UAVs){
      std::cout << i.getPosition()<<std::endl;
    }
    ros::Rate(5.0f).sleep();
}

/**
 * Calculate euclidean distance between two vectors.
 * @param v1 First vector.
 * @param v2 Second vector.
 * @return Distance of the two given vectors.
 */
// float UAVController::dist(Eigen::Vector3f v1, Eigen::Vector3f v2) {
//     return (v1-v2).norm();
// }

/**
 * Normalize vector. 
 * @param vec Vector to be normalized.
 * @return Normalized vector.
 */
//Eigen::Vector3f UAVController::normalize(Eigen::Vector3f vec) {
//    float m = vec.norm();
//    if (m > 1e-15 && m != 1) {
//        vec /= m;
//    }
//    return vec;
//}

/**
 * Fits an exponential function to the input data
 * @param x X-axis of the input data
 * @param y Y-axis of the input data
 * @return std::vector<double> of the exponential parameters (a,b) for f(x)=a*e^(b*x)
 */
std::vector<double> fit(std::vector<double> x, std::vector<double> y){
    unsigned long n = x.size();
    double lny[n], a, b, c;
    for (int i=0; i<n; i++) lny[i] = log(y[i]);

    double xsum=0, x2sum=0, ysum=0, xysum=0;    //variables for sums/sigma of xi,yi,xi^2,xiyi etc
    for (int i=0; i<n; i++){
        xsum = xsum+x[i];             //calculate sigma(xi)
        ysum = ysum+lny[i];           //calculate sigma(yi)
        x2sum = x2sum+pow(x[i],2);    //calculate sigma(x^2i)
        xysum = xysum+x[i]*lny[i];    //calculate sigma(xi*yi)
    }
    a = (n*xysum-xsum*ysum)/(n*x2sum-xsum*xsum);        //calculate slope(or the the power of exp)
    b = (x2sum*ysum-xsum*xysum)/(x2sum*n-xsum*xsum);    //calculate intercept
    c = pow(2.71828, b);                                //since b=ln(c)

    double y_fit[n];
    for (int i=0;i<n;i++) y_fit[i] = c*pow(2.71828, a*x[i]);    //to calculate y(fitted) at given x points

    std::vector<double> out = {c, a};
    return out;
}