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

  update_meas_pkg(sensor_measurement);

  add_ground_truth();

  // Call ProcessMeasurement(meas_package) for Kalman filter
  fusionEKF.ProcessMeasurement(meas_package);

  // Push the current estimated x,y positon from the Kalman filter's state vector
  add_estimate();

  VectorXd rmse = tools.CalculateRMSE(estimations, ground_truth);

  std::string msg = GetOutputMessageString(rmse);

  network->SendMessageToSimulator(msg);
}


std::string Controller::GetOutputMessageString(const Eigen::VectorXd rmse){
  nlohmann::json msgJson;
  msgJson["estimate_x"] = fusionEKF.ekf_.x_(0);
  msgJson["estimate_y"] = fusionEKF.ekf_.x_(1);
  msgJson["rmse_x"] = rmse(0);
  msgJson["rmse_y"] = rmse(1);
  msgJson["rmse_vx"] = rmse(2);
  msgJson["rmse_vy"] = rmse(3);
  auto msg = "42[\"estimate_marker\"," + msgJson.dump() + "]";
  return msg;
}

void Controller::add_estimate(){
  VectorXd estimate(4);
  estimate(0) = fusionEKF.ekf_.x_(0);
  estimate(1) = fusionEKF.ekf_.x_(1);
  estimate(2) = fusionEKF.ekf_.x_(2);
  estimate(3) = fusionEKF.ekf_.x_(3);
  estimations.push_back(estimate);
}

void Controller::add_ground_truth(){
  VectorXd gt_values(4);
  gt_values(0) = meas_package.x_ground_truth_;
  gt_values(1) = meas_package.y_ground_truth_;
  gt_values(2) = meas_package.vx_ground_truth_;
  gt_values(3) = meas_package.vy_ground_truth_;
  ground_truth.push_back(gt_values);
}

void Controller::update_meas_pkg(const string &sensor_measurement){
  std::istringstream iss(sensor_measurement);
  // reads first element from the current line
  string sensor_type;
  iss >> sensor_type;
  if (sensor_type == "L") {
    meas_package.sensor_type_ = MeasurementPackage::LASER;
    meas_package.raw_measurements_ = VectorXd(2);
    float px;
    float py;
    iss >> px;
    iss >> py;
    meas_package.raw_measurements_ << px, py;
  } else if (sensor_type == "R") {
    meas_package.sensor_type_ = MeasurementPackage::RADAR;
    meas_package.raw_measurements_ = VectorXd(3);
    float ro;
    float theta;
    float ro_dot;
    iss >> ro;
    iss >> theta;
    iss >> ro_dot;
    meas_package.raw_measurements_ << ro, theta, ro_dot;
  }
  iss >> meas_package.timestamp_;
  iss >> meas_package.x_ground_truth_;
  iss >> meas_package.y_ground_truth_;
  iss >> meas_package.vx_ground_truth_;
  iss >> meas_package.vy_ground_truth_;
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

