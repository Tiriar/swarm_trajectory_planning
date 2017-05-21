#include "../headers/data_fitting.h"

using namespace std;

//uav_safe_distance = 2, obs_safe_distance = 2, obs_critical_distance = 1, obstacle radius = 10
double time_without_obstacles = 1.584; // normalized to 1 meter!
// x[i] = gap size for i-th measurement
vector<double> x = {1.0, 1.2, 1.4, 1.6, 1.8, 2.0, 2.2, 2.4, 2.6, 2.8, 3.0, 3.5, 4.0};
// y[FLOCK_SIZE][i] = measured time increment for i-th gap size (normalized to 1 meter!)
vector<vector<double>> y = {{5.700, 4.796, 4.200, 3.998, 3.602, 3.298, 3.008, 2.896, 2.685, 2.433, 2.381, 2.022, 1.809},
                            {7.314, 6.698, 6.138, 5.752, 5.298, 4.998, 4.314, 4.092, 3.800, 3.396, 3.094, 2.468, 1.998},
                            {9.515, 8.836, 7.951, 7.536, 7.048, 6.521, 6.093, 5.684, 5.281, 4.935, 4.729, 4.112, 3.598}};

/**
 * Computes the edges evaluation function and prints it out (d = distance, g = gap size, f = flock size)
 */
void fitting_run() {
    unsigned long n = y.size();

    vector<double> exps;
    vector<double> exp_a;
    vector<double> exp_b;
    vector<double> flock_size;
    for (int i=0; i<n; i++) {
        exps = exp_fit(x, y[i]);
        if (i>0) {
            exp_a.push_back(exps[0]);
            exp_b.push_back(exps[1]);
            flock_size.push_back((double) (i+1));
        } else {
            cout << "Function for 1 UAV: " << time_without_obstacles
                 << "*d+" << exps[0] << "*e^(" << exps[1] << "*g)" << endl;
        }
    }

    vector<double> lin_a = lin_fit(flock_size, exp_a);
    vector<double> lin_b = lin_fit(flock_size, exp_b);
    cout << "Function for multiple UAVs: " << time_without_obstacles
         << "*d+(" << lin_a[0] << "*f+" << lin_a[1]
         <<")*e^((" << lin_b[0] << "*f+" << lin_b[1] << ")*g)" << endl;
}

/**
 * Fits a linear function to the input data
 * @param x X-axis of the input data
 * @param y Y-axis of the input data
 * @return std::vector<double> of the exponential parameters (a,b) for f(x)=ax+b
 */
vector<double> lin_fit(vector<double> x, vector<double> y) {
    unsigned long n = x.size();
    double a, b;

    double xsum=0, x2sum=0, ysum=0, xysum=0;    //variables for sums/sigma of xi,yi,xi^2,xiyi etc
    for (int i=0; i<n; i++) {
        xsum = xsum+x[i];           //calculate sigma(xi)
        ysum = ysum+y[i];           //calculate sigma(yi)
        x2sum = x2sum+pow(x[i],2);  //calculate sigma(x^2i)
        xysum = xysum+x[i]*y[i];    //calculate sigma(xi*yi)
    }
    a = (n*xysum-xsum*ysum)/(n*x2sum-xsum*xsum);        //calculate slope
    b = (x2sum*ysum-xsum*xysum)/(x2sum*n-xsum*xsum);    //calculate intercept

    double y_fit[n];
    for (int i=0; i<n; i++) y_fit[i] = a*x[i]+b;    //to calculate y(fitted) at given x points

    vector<double> out = {a, b};
    return out;
}

/**
 * Fits an exponential function to the input data
 * @param x X-axis of the input data
 * @param y Y-axis of the input data
 * @return std::vector<double> of the exponential parameters (a,b) for f(x)=a*e^(b*x)
 */
vector<double> exp_fit(vector<double> x, vector<double> y) {
    unsigned long n = x.size();
    double lny[n], a, b, c;
    for (int i=0; i<n; i++) lny[i] = log(y[i]);

    double xsum=0, x2sum=0, ysum=0, xysum=0;    //variables for sums/sigma of xi,yi,xi^2,xiyi etc
    for (int i=0; i<n; i++) {
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

    vector<double> out = {c, a};
    return out;
}
