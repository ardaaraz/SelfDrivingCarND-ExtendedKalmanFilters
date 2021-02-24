#include "FusionEKF.h"
#include <iostream>
#include "Eigen/Dense"
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::vector;

/**
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
              0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
              0, 0.0009, 0,
              0, 0, 0.09;

  /**
   * TODO: Finish initializing the FusionEKF.
   * TODO: Set the process and measurement noises
   */

  // Initialize state covariance matrix P
  ekf_.P_ = MatrixXd(4, 4);
  ekf_.P_ << 1, 0,    0,    0,
             0, 1,    0,    0,
             0, 0, 1000,    0,
             0, 0,    0, 1000;

  // Initialize state transition matrix F
  ekf_.F_ = MatrixXd(4, 4);
  ekf_.F_ << 1, 0, 1, 0,
             0, 1, 0, 1,
             0, 0, 1, 0,
             0, 0, 0, 1;

  // Initialize measurement matrix H for laser measurements
  H_laser_ << 1, 0, 0, 0,
              0, 1, 0, 0;

  //Initialize process noise covariance matrix Q
  ekf_.Q_ = MatrixXd::Zero(4, 4);
  std::cout << "EKF is constructed!" << std::endl;
}

/**
 * Destructor.
 */
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  /**
   * Initialization
   */
  if (!is_initialized_) {
    /**
     * TODO: Initialize the state ekf_.x_ with the first measurement.
     * TODO: Create the covariance matrix.
     * You'll need to convert radar from polar to cartesian coordinates.
     */

    // first measurement
    cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 1, 1;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      // TODO: Convert radar from polar to cartesian coordinates 
      //         and initialize state.
      ekf_.x_(0) = measurement_pack.raw_measurements_(0) * cos(measurement_pack.raw_measurements_(1));
      ekf_.x_(1) = measurement_pack.raw_measurements_(0) * sin(measurement_pack.raw_measurements_(1));
      ekf_.x_(2) = measurement_pack.raw_measurements_(2) * cos(measurement_pack.raw_measurements_(1));
      ekf_.x_(3) = measurement_pack.raw_measurements_(2) * sin(measurement_pack.raw_measurements_(1));
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      // TODO: Initialize state.
      ekf_.x_(0) = measurement_pack.raw_measurements_(0); 
      ekf_.x_(1) = measurement_pack.raw_measurements_(1);
      ekf_.x_(2) = 0.0; 
      ekf_.x_(3) = 0.0;
    }
    // Update time stamp
    previous_timestamp_ = measurement_pack.timestamp_;

    // done initializing, no need to predict or update
    is_initialized_ = true;
    std::cout << "Initialization is completed!" << std::endl;
    return;
  }

  /**
   * Prediction
   */

  /**
   * TODO: Update the state transition matrix F according to the new elapsed time.
   * Time is measured in seconds.
   * TODO: Update the process noise covariance matrix.
   * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */

  // compute the time elapsed between the current and previous measurements
  // dt - expressed in seconds
  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
  previous_timestamp_ = measurement_pack.timestamp_;

  // Update state transition matrix F
  ekf_.F_(0, 2) = dt;
  ekf_.F_(1, 3) = dt;

  // Update process noise covariance matrix Q
  const float noise_ax = 9.0;
  const float noise_ay = 9.0;

  ekf_.Q_ <<  pow(dt, 4) / 4 * noise_ax, 0                        , pow(dt, 3) / 2 * noise_ax, 0                        ,
              0                        , pow(dt, 4) / 4 * noise_ay, 0                        , pow(dt, 3) / 2 * noise_ay,
              pow(dt, 3) / 2 * noise_ax, 0                        , pow(dt, 2) * noise_ax    , 0                        ,
              0                        , pow(dt, 3) / 2 * noise_ay, 0                        , pow(dt, 2) * noise_ay    ;
  
  std::cout << "Q = " << ekf_.Q_ << std::endl;
  ekf_.Predict();
  std::cout << "Prediction!" << std::endl;
  std::cout << "Measurement Type = " << measurement_pack.sensor_type_ << std::endl;

  /**
   * Update
   */

  /**
   * TODO:
   * - Use the sensor type to perform the update step.
   * - Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // TODO: Radar updates
    
    // Calculate Jacobian of measurement matrix H
    Hj_ = tools.CalculateJacobian(ekf_.x_);
    std::cout << "Jacobian is calculated!" << std::endl;
    // Set measurement matrix H
    ekf_.H_ = Hj_;
    // Set measurement noise covariance matrix R
    ekf_.R_ = R_radar_;
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);
    std::cout << "Update with Radar Measurement!" << std::endl;

  } else {
    // TODO: Laser updates
    
    // Set measurement matrix H
    ekf_.H_ = H_laser_;
    // Set measurement noise covariance matrix R
    ekf_.R_ = R_laser_;
    // Call update function
    ekf_.Update(measurement_pack.raw_measurements_);
    std::cout << "Update with Laser Measurement!" << std::endl;
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
