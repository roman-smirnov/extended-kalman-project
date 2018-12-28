/*
================================================================================================================================

 plan_controller.h

 Management, logic, message processing and output.

================================================================================================================================
*/

#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <json.hpp>
#include <math.h>
#include <iostream>
#include "kalman/FusionEKF.h"
#include "kalman/tools.h"
#include "network.h"

namespace ekf {

class Network;

class Controller {
 public:
  Controller();
  virtual ~Controller();

  // parse and handle messages received from simulator
  void HandleSimulatorMessage(char *data, size_t length);

  // set the gateways to handle network
  void SetNetworkGateway(Network *network);

 private:

  // receive requests and send messages
  Network * network = nullptr;
  MeasurementPackage meas_package;
  FusionEKF fusionEKF;
  std::vector<Eigen::VectorXd> estimations;
  std::vector<Eigen::VectorXd> ground_truth;
  Tools tools;

  // Checks if event has JSON data. return the message in strin format, or return an empty string if there's no message
  std::string has_data(std::string msg_str);

  void update_meas_pkg(const std::string &sensor_measurement);
  void add_ground_truth();
  void add_estimate();
  std::string GetOutputMessageString(const Eigen::VectorXd rmse);
};

}
#endif //CONTROLLER_H
