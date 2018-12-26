#include "tools.h"
#include <iostream>
#include <math.h>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() = default;

Tools::~Tools() = default;

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {

  VectorXd rmse(4);
  rmse << 0, 0, 0, 0;
  // estimation vector size should not be zero
  if (estimations.empty()) {
    std::cout << "Invalid estimation - should not be zero" << std::endl;
    return rmse;
  }

  //  estimation vector size should equal ground truth vector size
  if (estimations.size() != ground_truth.size()) {
    std::cout << "Invalid estimation - size should equal ground truth vector size" << std::endl;
    return rmse;
  }

  // sum of squared differences
  for (unsigned int i = 0; i < estimations.size(); ++i) {
    // difference
    VectorXd residual = estimations[i] - ground_truth[i];
    // square of difference
    residual = residual.array().square();
    // sum
    rmse += residual;
  }

  // mean of squared differences
  rmse = rmse / estimations.size();

  // square root of mean squared differences
  rmse = rmse.array().sqrt();

  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd &state) {
  MatrixXd Hj(3, 4);

  //capture state parameters
  double x = state(0);
  double y = state(1);
  double vx = state(2);
  double vy = state(3);

  //pre-compute matrix terms
  double c1 = pow(x, 2) + pow(y, 2);
  double c2 = sqrt(c1);
  double c3 = c1 * c2;

  //avoid division by zero
  if (fabs(c1) < 0.0001) {
    Hj << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
    return Hj;
  }

  //compute the Jacobian matrix
  Hj << (x / c2), (y / c2), 0, 0,
      -(y / c1), (x / c1), 0, 0,
      y * (vx * y - vy * x) / c3, x * (x * vy - y * vx) / c3, x / c2, y / c2;

  return Hj;
}
