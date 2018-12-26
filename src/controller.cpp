/*
================================================================================================================================

 plan_controller.cpp

 Implementation of class Controller declared in plan_controller.h

================================================================================================================================
*/

#include "controller.h"

namespace ekf {

using std::string;
using std::vector;
using std::cout;
using std::cerr;
using std::endl;
using Eigen::MatrixXd;
using Eigen::VectorXd;

Controller::Controller() = default;
Controller::~Controller() = default;


void Controller::SetNetworkGateway(Network *network) {
  this->network = network;
}

void Controller::HandleSimulatorMessage(char *data, size_t length) {
  if (network == nullptr) {
    return;
  }

  if (length <= 2 || data[0] != '4' || data[1] != '2') {
    return;
  }

  auto msg_str = has_data(data);
  
  if (msg_str.empty()) {
    string msg = "42[\"manual\",{}]"; // Manual driving
    network->SendMessageToSimulator(msg);
    return;
  }

  auto j = nlohmann::json::parse(msg_str);
  auto event = j[0].get<string>();

  if (event != "telemetry") {
    return;
  }

  // j[1] is the data JSON object
  string sensor_measurement = j[1]["sensor_measurement"];

  MeasurementPackage meas_package;
  std::istringstream iss(sensor_measurement);

  long long timestamp;

  // reads first element from the current line
  string sensor_type;
  iss >> sensor_type;

  if (sensor_type.compare("L") == 0) {
    meas_package.sensor_type_ = MeasurementPackage::LASER;
    meas_package.raw_measurements_ = VectorXd(2);
    float px;
    float py;
    iss >> px;
    iss >> py;
    meas_package.raw_measurements_ << px, py;
    iss >> timestamp;
    meas_package.timestamp_ = timestamp;
  } else if (sensor_type.compare("R") == 0) {
    meas_package.sensor_type_ = MeasurementPackage::RADAR;
    meas_package.raw_measurements_ = VectorXd(3);
    float ro;
    float theta;
    float ro_dot;
    iss >> ro;
    iss >> theta;
    iss >> ro_dot;
    meas_package.raw_measurements_ << ro, theta, ro_dot;
    iss >> timestamp;
    meas_package.timestamp_ = timestamp;
  }

  float x_gt;
  float y_gt;
  float vx_gt;
  float vy_gt;
  iss >> x_gt;
  iss >> y_gt;
  iss >> vx_gt;
  iss >> vy_gt;

  // Create a Kalman Filter instance
  FusionEKF fusionEKF;

  // used to compute the RMSE later
  Tools tools;
  vector<VectorXd> estimations;
  vector<VectorXd> ground_truth;

  VectorXd gt_values(4);
  gt_values(0) = x_gt;
  gt_values(1) = y_gt;
  gt_values(2) = vx_gt;
  gt_values(3) = vy_gt;
  ground_truth.push_back(gt_values);

  // Call ProcessMeasurement(meas_package) for Kalman filter
  fusionEKF.ProcessMeasurement(meas_package);

  // Push the current estimated x,y positon from the Kalman filter's
  //   state vector

  VectorXd estimate(4);

  double p_x = fusionEKF.ekf_.x_(0);
  double p_y = fusionEKF.ekf_.x_(1);
  double v1 = fusionEKF.ekf_.x_(2);
  double v2 = fusionEKF.ekf_.x_(3);

  estimate(0) = p_x;
  estimate(1) = p_y;
  estimate(2) = v1;
  estimate(3) = v2;

  estimations.push_back(estimate);

  VectorXd RMSE = tools.CalculateRMSE(estimations, ground_truth);

  nlohmann::json msgJson;
  msgJson["estimate_x"] = p_x;
  msgJson["estimate_y"] = p_y;
  msgJson["rmse_x"] = RMSE(0);
  msgJson["rmse_y"] = RMSE(1);
  msgJson["rmse_vx"] = RMSE(2);
  msgJson["rmse_vy"] = RMSE(3);
  auto msg = "42[\"estimate_marker\"," + msgJson.dump() + "]";
  // std::cout << msg << std::endl;
  network->SendMessageToSimulator(msg);
}

string Controller::has_data(string msg_str) {
  auto found_null = msg_str.find("null");
  auto b1 = msg_str.find_first_of('[');
  auto b2 = msg_str.find_first_of('}');
  if (found_null != string::npos) {
    return "";
  } else if (b1 != std::string::npos && b2 != std::string::npos) {
    return msg_str.substr(b1, b2 - b1 + 2);
  }
  return "";
}

}

