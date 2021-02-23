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
  Hj << px / sqrt(pow(px, 2) + pow(py, 2))                            , py / sqrt(pow(px, 2) + pow(py, 2))                            , 0                                 , 0                                 , 
        -py / (pow(px, 2) + pow(py, 2))                               , px / (pow(px, 2) + pow(py, 2))                                , 0                                 , 0                                 ,
        py * (vx * py - vy * px) / pow((pow(px, 2) + pow(py, 2)), 1.5), px * (vy * px - vx * py) / pow((pow(px, 2) + pow(py, 2)), 1.5), px / sqrt(pow(px, 2) + pow(py, 2)), py / sqrt(pow(px, 2) + pow(py, 2));
  return Hj;
}
