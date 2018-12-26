/*
================================================================================================================================

 networking module

================================================================================================================================
*/


#ifndef NETWORK_H
#define NETWORK_H

#include <uWS/Hub.h>
#include <vector>
#include <string>
#include <cmath>
#include <iostream>
#include "controller.h"

namespace ekf {

class Controller;

// handles simulator and prolog networking via websockets
class Network {
 public:
  // sets up the networking component
  Network(Controller &controller);

  virtual ~Network();

  // start the server, begin handling requests and responses, etc
  void StartServer();

  // set the controller to handle request callbacks
//  void SetController(const Controller &controller);

  // send a message to the simulator
  void SendMessageToSimulator(std::string &msg);

 private:

  void InitCallbacks();
  void InitListening();

  // simulator client localhost port
  static constexpr int SIMULATOR_CLIENT_PORT = 4567;

  // management and logic object, processes incoming messages and issues output commands.
  Controller &controller;

  // websocket handling and managment
  uWS::Hub websocket_hub;

};

}
#endif //NETWORK_H
