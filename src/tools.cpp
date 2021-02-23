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

  MatrixXd Hj(3,4);
  // recover state parameters
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);
  // check division by zero
  if (fabs(pow(px, 2) + pow(py, 2)) < 0.0001) 
  {
     std::cout << "CalculateJacobian () - Error - Division by Zero" << std::endl;
     return Hj;
  }
  Hj << px / sqrt(pow(px, 2) + pow(py, 2))                            , py / sqrt(pow(px, 2) + pow(py, 2))                            , 0                                 , 0                                 , 
        -py / (pow(px, 2) + pow(py, 2))                               , px / (pow(px, 2) + pow(py, 2))                                , 0                                 , 0                                 ,
        py * (vx * py - vy * px) / pow((pow(px, 2) + pow(py, 2)), 1.5), px * (vy * px - vx * py) / pow((pow(px, 2) + pow(py, 2)), 1.5), px / sqrt(pow(px, 2) + pow(py, 2)), py / sqrt(pow(px, 2) + pow(py, 2));
  return Hj;
}
