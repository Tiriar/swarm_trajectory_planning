#include "../headers/UAVController.h"

using namespace std;
/**
 * Constructor
 * - sets subscribers
 * @param nh ros NodeHandle
 */
UAVController::UAVController(const ros::NodeHandle& nh, const int &uavCount): nh_(nh){
  startPosition = Eigen::Vector3f(10.0f, 0.0f, 0.0f);
  goalPosition = Eigen::Vector3f(20.0f, 0.0f, 0.0f);
  for(int i=1;i<uavCount+1;i++){
    std::ostringstream uavName;
    uavName << "/uav" << i<<"/mbzirc_odom/slow_odom";
    std::cout << "Creating "<<uavName.str().c_str()<<std::endl;
    UAVs.push_back(new UAV(nh_,uavName.str().c_str()));
  }
}


UAVController::~UAVController() {
//   for(UAV *uav : UAVs){
//     delete &uav;
//   }
}

/**
 * Method to run the controller with. Definitely or indefinitely, based on config variable.
 */
void UAVController::run() {
  while (ros::ok() && !uavsInCircle(startPosition, RADIUS)) {
    for(UAV *i :UAVs){
      Eigen::Vector3f pos = i->getPosition();
//       printf("%1.3f, %1.3f, %1.3f\n", pos[0], pos[1], pos[2]);
    }
    ros::Rate(0.5f).sleep();
  }
  measurementStart = ros::Time::now().toSec();
  ROS_INFO("MEASUREMENT START");
  while (ros::ok() && !uavsInCircle(goalPosition, RADIUS)) {
    for(UAV *i :UAVs){
      Eigen::Vector3f pos = i->getPosition();
//       printf("%1.3f, %1.3f, %1.3f\n", pos[0], pos[1], pos[2]);
    }
    ros::Rate(5.0f).sleep();
  }
  measurementEnd = ros::Time::now().toSec();
  ROS_INFO("MEASUREMENT END");
  
  cout << "FLIGHT DURATION: "<<(measurementEnd - measurementStart) << endl;
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

float UAVController::dist2D(Eigen::Vector3f v1, Eigen::Vector3f v2){
    Eigen::Vector2f t1 = Eigen::Vector2f(v1[0], v1[1]);
    Eigen::Vector2f t2 = Eigen::Vector2f(v2[0], v2[1]);
    return (t1-t2).norm();
    
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

//True if all uavs within cylinder of radius r at position pos
bool UAVController::uavsInCircle(Eigen::Vector3f pos, float r){
  for(UAV *u : UAVs){
    if(dist2D(u->getPosition(),pos)>RADIUS){
      return true;
    }
  }
  return false;
}

/**
 * Fits an exponential function to the input data
 * @param x X-axis of the input data
 * @param y Y-axis of the input data
 * @return std::vector<double> of the exponential parameters (a,b) for f(x)=a*e^(b*x)
 */
std::vector<double> UAVController::fit(std::vector<double> x, std::vector<double> y){
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
