#include "kalman_filter.h"
#include <iostream>
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

/* 
 * Please note that the Eigen library does not initialize 
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in_laser, MatrixXd &R_in_radar,
                        MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_laser_ = R_in_laser;
  R_radar_ = R_in_radar;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  /**
   * TODO: predict the state
   */
  x_ = F_ * x_;
  P_ = F_ * P_ * F_.transpose() + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
   * TODO: update the state by using Kalman Filter equations
   */
  VectorXd y = z - H_ * x_;
  MatrixXd S = H_ * P_ * H_.transpose() + R_laser_;
  MatrixXd K = P_ * H_.transpose() * S.inverse();
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  x_ = x_ + K * y;
  P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   * TODO: update the state by using Extended Kalman Filter equations
   */
  // define Hj using the x vector values
  Tools tools;
  MatrixXd Hj(3, 4);
  Hj << tools.CalculateJacobian(x_);
 
  // derive the new h(x) as well
  float px = x_[0];
  float py = x_[1];
  float vx = x_[2];
  float vy = x_[3];
  float pi = atan(1)*4;
  float rtsumsquare = pow(px*px + py*py, 0.5);
  VectorXd hx(3);
  hx << rtsumsquare, atan2(py, px), (px*vx + py*vy)/rtsumsquare;
  
  // adjust rho in y such that its between -pi and pi
  VectorXd y = z - hx;
  if (y[1] < -0.5*pi) { y[1] = y[1] + 2*pi; }
  if (y[1] > 0.5*pi) { y[1] = y[1] - 2*pi; }
  
  // combine with Hj
  MatrixXd S = Hj * P_ * Hj.transpose() + R_radar_;
  MatrixXd K = P_ * Hj.transpose() * S.inverse();
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  
  // update
  x_ = x_ + K * y;
  P_ = (I - K * Hj) * P_;
}
