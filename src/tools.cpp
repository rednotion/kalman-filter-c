#include "tools.h"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
   * TODO: Calculate the RMSE here.
   */
  VectorXd rmse(4);
  rmse << 0,0,0,0;

  // check the validity of the following inputs:
  //  * the estimation vector size should not be zero
  //  * the estimation vector size should equal ground truth vector size
  if (estimations.size() != ground_truth.size()
      || estimations.size() == 0) {
    std::cout << "Invalid estimation or ground_truth data" << std::endl;
    return rmse;
  }

  // accumulate squared residuals
  for (unsigned int i=0; i < estimations.size(); ++i) {

    VectorXd residual = estimations[i] - ground_truth[i];

    // coefficient-wise multiplication
    residual = residual.array()*residual.array();
    rmse += residual;
  }

  // calculate the mean
  rmse = rmse/estimations.size();

  // calculate the squared root
  rmse = rmse.array().sqrt();

  // return the result
  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
   * TODO:
   * Calculate a Jacobian here.
   */
  MatrixXd Hj(4, 4);
  // Hj = MatrixXd(4, 4);
  
  float px = x_state[0];
  float py = x_state[1];
  float vx = x_state[2];
  float vy = x_state[3];
  float sumsquare = px*px + py*py;
  
  if (fabs(sumsquare) < 0.0001) {
    std::cout << "CalculateJacobian () - Error - Division by Zero" << std::endl;
    sumsquare = 0.0001; // set to 0.0001 to prevent division by 0, then continue
  }
  
  float rtsumsquare = pow(sumsquare, 0.5);
  float ttsumsquare = pow(sumsquare, 1.5);
  
  Hj << px/rtsumsquare, py/rtsumsquare, 0, 0,
        -py/sumsquare, px/sumsquare, 0, 0,
        py*(vx*py - vy*px)/ttsumsquare, px*(vy*px - vx*py)/ttsumsquare, px/rtsumsquare, py/rtsumsquare;
  
  return Hj;
}
