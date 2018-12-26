/*
================================================================================================================================

 plan_controller.h

 Management, logic, message processing and output.

================================================================================================================================
*/

#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "json.hpp"
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

  // set the gateways to handle networking
  void SetNetworkGateway(Network *network);

 private:

  // receive requests and send messages
  Network * network = nullptr;

  // Checks if event has JSON data. return the message in strin format, or return an empty string if there's no message
  std::string has_data(std::string msg_str);

};

}
#endif //CONTROLLER_H
