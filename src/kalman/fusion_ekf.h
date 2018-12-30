#ifndef FusionEKF_H_
#define FusionEKF_H_

#include "../precompiled.h"
#include "kalman_filter.h"
#include "measurement_package.h"
#include "tools.h"

class FusionEKF {
 public:
  /**
   * Constructor.
   */
  FusionEKF();

  /**
   * Destructor.
   */
  virtual ~FusionEKF();

  /**
   * Run the whole flow of the Kalman Filter from here.
   */
  void ProcessMeasurement(const MeasurementPackage &measurement_pack);

  /**
   * Kalman Filter update and prediction math lives in here.
   */
  KalmanFilter ekf_;

 private:
  // check whether the tracking toolbox was initialized or not (first measurement)
  bool is_initialized_;

  // previous timestamp
  long long previous_timestamp_;

  // tool object used to comp_ute Jacobian and RMSE
  Tools tools;
  Eigen::MatrixXd R_laser_;
  Eigen::MatrixXd R_radar_;
  Eigen::MatrixXd H_laser_;
  Eigen::MatrixXd H_radar_;

  // acceleration noise
  double acceleration_noise_;

  void InitStateOnFirstMeasurement(const MeasurementPackage &measurement_pack);
  void ComputeProcessCovMatrix(const MeasurementPackage &measurement_pack);
  void UpdateFusionKalmanFilter(const MeasurementPackage &measurement_pack);
};

#endif // FusionEKF_H_
