#include "kalman_filter.h"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

/* 
 * Please note that the Eigen library does not initialize 
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
  std::cout << "Kalman Filter init" << std::endl;
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

  // Number of state
  const int num_state = x_.size();
  // Identity matrix
  MatrixXd I = MatrixXd::Identity(num_state, num_state);

  MatrixXd y = z - H_ * x_;
  MatrixXd S = H_ * P_ * H_.transpose() + R_;
  MatrixXd K = P_ * H_.transpose() * S.inverse();
  x_ = x_ + K * y;
  P_ = (I - K * H_) * P_;

}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   * TODO: update the state by using Extended Kalman Filter equations
   */

  // Number of state
  const int num_state = x_.size();
  // Identity matrix
  MatrixXd I = MatrixXd::Identity(num_state, num_state);

  // Calculate h vector at x_
  MatrixXd h = MatrixXd(3, 1);
  h(0, 0) = sqrt(x_(0) * x_(0) + x_(1) * x_(1));
  h(1, 0) = atan2(x_(1), x_(0));
  h(2, 0) = (x_(0) * x_(2) + x_(1) * x_(3)) / sqrt(x_(0) * x_(0) + x_(1) * x_(1));

  MatrixXd y = z - h;
  // Normalize y
  while(y(1) > M_PI)
  {
    y(1) -= M_PI;
  }

  while(y(1) < -M_PI)
  {
    y(1) += M_PI;
  }
  MatrixXd S = H_ * P_ * H_.transpose() + R_;
  MatrixXd K = P_ * H_.transpose() * S.inverse();
  x_ = x_ + K * y;
  P_ = (I - K * H_) * P_;
}
