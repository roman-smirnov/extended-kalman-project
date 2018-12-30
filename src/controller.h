/*
================================================================================================================================

 plan_controller.h

 Management, logic, message processing and output.

================================================================================================================================
*/

#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "kalman/fusion_ekf.h"
#include "kalman/tools.h"

namespace kalman {

class Controller {
 public:
  Controller();
  virtual ~Controller();

  // parse and handle messages received from simulator
  void HandleSimulatorMessage(const char *data, size_t length);

  // register callback function by way of which send messages to simulator
  void RegisterSendMessageHandler(std::function<void(const std::string&)> SendMessageHandler);

 private:

  MeasurementPackage meas_package;
  FusionEKF fusionEKF;

  // Checks if event has JSON data. return the message in strin format, or return an empty string if there's no message
  std::string has_data(std::string msg_str);

  void update_meas_pkg(const std::string &sensor_measurement);

  std::string GetOutputMessageString(Eigen::VectorXd rmse);

  // callback function to send messages to the simulator
  std::function<void(const std::string&)> SendMessageToSimulator = nullptr;
};

}
#endif //CONTROLLER_H
