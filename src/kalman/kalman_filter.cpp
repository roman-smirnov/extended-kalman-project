#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

/* 
 * Please note that the Eigen library does not initialize 
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

KalmanFilter::KalmanFilter() = default;

KalmanFilter::~KalmanFilter() = default;

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {

  VectorXd z_pred = H_ * x_;

  VectorXd y = z - z_pred;
  MatrixXd Ht = H_.transpose();
  MatrixXd PHt = P_ * Ht;
  MatrixXd S = H_ * PHt + R_;
  MatrixXd Si = S.inverse();
  MatrixXd K = PHt * Si;

  //new estimate
  x_ = x_ + (K * y);
  P_ = (I_ - K * H_) * P_;

}

VectorXd CartesianToPolar(const VectorXd &state) {

  double x = state[0];
  double y = state[1];
  double vx = state[2];
  double vy = state[3];

  double rho = sqrt(x * x + y * y);
  double phi = atan2(y, x);  // returns values between -pi and pi
  double rho_dot = (rho < 0.000001) ? 0.000001 : (x * vx + y * vy)/rho; // avoid division by 0 in computing rho_dot

  VectorXd z_pred = VectorXd(3);
  z_pred << rho, phi, rho_dot;
  return z_pred;

}

void KalmanFilter::UpdateEKF(const VectorXd &z) {


  VectorXd z_pred = CartesianToPolar(x_);
  VectorXd y = z - z_pred;

  // normalize to [-pi, pi]
  y(1) -= TWO_PI * (int)(y(1)/TWO_PI);

  // following is exact the same as in the function of KalmanFilter::Update()
  MatrixXd Ht = H_.transpose();
  MatrixXd PHt = P_ * Ht;
  MatrixXd S = H_ * PHt + R_;
  MatrixXd Si = S.inverse();
  MatrixXd K = PHt * Si;

  //new estimate
  x_ = x_ + (K * y);
  P_ = (I_ - K * H_) * P_;
}
