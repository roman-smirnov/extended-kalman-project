/*
================================================================================================================================

 network module

================================================================================================================================
*/


#ifndef NETWORK_H
#define NETWORK_H

#include <uWS/Hub.h>
#include <string>
#include <iostream>

namespace kalman {

// handles simulator and prolog network via websockets
class Network {
 public:
  // sets up the network component
  Network();

  virtual ~Network();

  // start the server, begin handling requests and responses, etc
  void StartServer();

  // set controller function to handle incoming messages
  void RegisterReceiveMessageHandler(const std::function<void(char*, size_t)> HandleSimulatorMessage);

  // send a message to the simulator
  void SendMessageToSimulator(std::string msg);

 private:

  void InitCallbacks();
  void InitListening();

  // simulator client localhost port
  static constexpr int SIMULATOR_CLIENT_PORT = 4567;

  // callback function to handle messages received from simulator

  std::function<void(char *data, size_t length)> HandleSimulatorMessage = nullptr;

  // websocket handling and managment
  uWS::Hub websocket_hub;

};

}
#endif //NETWORK_H
