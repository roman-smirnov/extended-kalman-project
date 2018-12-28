#ifndef MEASUREMENT_PACKAGE_H_
#define MEASUREMENT_PACKAGE_H_

#include <Eigen/Dense>

class MeasurementPackage {
 public:
  enum SensorType{
    LASER,
    RADAR
  } sensor_type_;

  long long timestamp_;

  Eigen::VectorXd raw_measurements_;

  float x_ground_truth_;
  float y_ground_truth_;
  float vx_ground_truth_;
  float vy_ground_truth_;

};

#endif // MEASUREMENT_PACKAGE_H_
