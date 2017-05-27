#include "../headers/data_fitting.h"

/** \file
    \brief Contains functions required for evaluating the graph.
    
    Contains functions required for evaluating the graph using a heuristic function.
*/

using namespace std;

/**
 * Computes the edges evaluation function and prints it out (d = distance, g = gap size, f = flock size)
 * @param time_without_obstacles measured time without obstacles normalized to 1 meter
 * @param x x[i] = gap size for i-th measurement
 * @param y y[FLOCK_SIZE][i] = measured time increment for i-th gap size (normalized to 1 meter)
 */
void fitting_run(double time_without_obstacles, vector<double> x, vector<vector<double>> y) {
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
         <<")*e^((" << lin_b[0] << "*f" << lin_b[1] << ")*g)" << endl;
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
