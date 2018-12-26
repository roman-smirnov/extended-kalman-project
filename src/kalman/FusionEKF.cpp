#include "FusionEKF.h"
#include "../Eigen/Dense"
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
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
  H_radar_ = MatrixXd(3, 4);

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
      0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
      0, 0.0009, 0,
      0, 0, 0.09;

  H_laser_ << 1, 0, 0, 0,
      0, 1, 0, 0;

  //  H_radar is calculated later as jacobian matrix

  acceleration_noise_ = 9.0;

  // process covariance matrix
  ekf_.P_ = MatrixXd(4, 4);
  ekf_.P_ << 1, 0, 0, 0,
      0, 1, 0, 0,
      0, 0, 1000, 0,
      0, 0, 0, 1000;

  // state transition matrix
  ekf_.F_ = MatrixXd(4, 4);
  ekf_.F_ << 1, 0, 1, 0,
      0, 1, 0, 1,
      0, 0, 1, 0,
      0, 0, 0, 1;

  // state vector
  ekf_.x_ = VectorXd(4);
  ekf_.x_ << 0, 0, 0, 0;

  // identity matrix
  ekf_.I_ = MatrixXd(4, 4);
  ekf_.I_ << 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1;
}

// Destructor
FusionEKF::~FusionEKF() = default;

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  if (!is_initialized_) {
    InitStateOnFirstMeasurement(measurement_pack);
    is_initialized_ = true;
    return;
  }
  ComputeProcessCovMatrix(measurement_pack);
  ekf_.Predict();
  UpdateFusionKalmanFilter(measurement_pack);

}

void FusionEKF::InitStateOnFirstMeasurement(const MeasurementPackage &measurement_pack) {
  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // convert from polar to cartesian coordinates
    double rho = measurement_pack.raw_measurements_[0];      // magnitude from origin
    double phi = measurement_pack.raw_measurements_[1];      // direction angle from x axis
    ekf_.x_ << rho * cos(phi), rho * sin(phi), 0, 0;  // x, y, vx, vy
  } else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
    ekf_.x_ << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0, 0; // x, y, vx, vy
  }
  previous_timestamp_ = measurement_pack.timestamp_;
}

void FusionEKF::ComputeProcessCovMatrix(const MeasurementPackage &measurement_pack) {
  // compute timestamp microseconds difference, convert to seconds
  double dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
  previous_timestamp_ = measurement_pack.timestamp_;
  double dt_2 = dt * dt;
  double dt_3 = dt_2 * dt;
  double dt_4 = dt_3 * dt;
  // integrate time differences into state transition matrix F
  ekf_.F_(0, 2) = dt;
  ekf_.F_(1, 3) = dt;
  // compute the process covariance matrix Q
  ekf_.Q_ = MatrixXd(4, 4);
  ekf_.Q_ << dt_4 / 4 * acceleration_noise_, 0, dt_3 / 2 * acceleration_noise_, 0,
      0, dt_4 / 4 * acceleration_noise_, 0, dt_3 / 2 * acceleration_noise_,
      dt_3 / 2 * acceleration_noise_, 0, dt_2 * acceleration_noise_, 0,
      0, dt_3 / 2 * acceleration_noise_, 0, dt_2 * acceleration_noise_;
}

void FusionEKF::UpdateFusionKalmanFilter(const MeasurementPackage &measurement_pack) {
  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    H_radar_ = tools.CalculateJacobian(ekf_.x_);
    ekf_.H_ = H_radar_;
    ekf_.R_ = R_radar_;
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);
  } else {
    ekf_.H_ = H_laser_;
    ekf_.R_ = R_laser_;
    ekf_.Update(measurement_pack.raw_measurements_);
  }
}


