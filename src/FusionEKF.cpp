#include "FusionEKF.h"
#include <iostream>
#include "Eigen/Dense"
#include "tools.h"
#include "cmath"

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

  //measurement matrix - laser
  H_laser_ << 1, 0, 0, 0,
              0, 1, 0, 0;

  //jacobian matrix - radar
  Hj_ = MatrixXd::Zero(3,4);

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
      double x = measurement_pack.raw_measurements_[0] * cos(measurement_pack.raw_measurements_[1]);
      double y = measurement_pack.raw_measurements_[0] * sin(measurement_pack.raw_measurements_[1]);
      double vx = measurement_pack.raw_measurements_[2] * cos(measurement_pack.raw_measurements_[1]);
      double vy = measurement_pack.raw_measurements_[2] * sin(measurement_pack.raw_measurements_[1]);

      ekf_.x_<< x, y, vx, vy;

      //Create the covariance matrix 
      ekf_.P_ = MatrixXd(4, 4);
      ekf_.P_ <<  1, 0, 0, 0,
                  0, 1, 0, 0,
                  0, 0, 10, 0,
                  0, 0, 0, 10; 
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      // TODO: Initialize state.
      // Initialize the state ekf_.x_ with the first measurement.
       ekf_.x_ << measurement_pack.raw_measurements_[0], 
              measurement_pack.raw_measurements_[1], 
              0, 
              0;

      //Create the covariance matrix 
      ekf_.P_ = MatrixXd(4, 4);
      ekf_.P_ <<  1, 0, 0, 0,
                  0, 1, 0, 0,
                  0, 0, 1000, 0,
                  0, 0, 0, 1000;   
    }

    // done initializing, no need to predict or update
    is_initialized_ = true;
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

  // 1. Modify the F matrix so that the time is integrated
  ekf_.F_ = MatrixXd(4,4);
  ekf_.F_ <<  1, 0, dt, 0,
              0, 1, 0, dt,
              0, 0, 1, 0,
              0, 0, 0, 1;

  // 2. Set the process covariance matrix Q
  float noise_ax = 9;
  float noise_ay = 9;

  float dt2 = dt*dt;
  float dt3 = (dt2 * dt) / 2;
  float dt4 = (dt3 * dt) /2 ;
  float noise_ax2 = noise_ax*noise_ax;
  float noise_ay2 = noise_ay*noise_ay;

  ekf_.Q_ = MatrixXd(4,4);
  ekf_.Q_ << dt4*noise_ax2, 0, dt3*noise_ax2, 0,
            0, dt4*noise_ay2, 0, dt3*noise_ay2,
            dt3*noise_ax2, 0, dt2*noise_ax2, 0,
            0, dt4*noise_ay2, 0, dt2*noise_ay2;

  ekf_.Predict();

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

    Hj_ = tools.CalculateJacobian(ekf_.x_);
    ekf_.H_ = Hj_;
    ekf_.R_ = R_radar_;

    ekf_.UpdateEKF(measurement_pack.raw_measurements_);

  } else {
    // TODO: Laser updates
    ekf_.H_ = H_laser_;
    ekf_.R_ = R_laser_;
    ekf_.Update(measurement_pack.raw_measurements_);

  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
