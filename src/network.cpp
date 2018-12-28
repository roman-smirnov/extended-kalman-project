/*
================================================================================================================================

 implementation of network module

================================================================================================================================
*/

#include "network.h"

namespace kalman {

using std::cout;
using std::cerr;
using std::string;
using std::endl;

Network::Network() = default;

Network::~Network() = default;

void Network::StartServer() {
  InitCallbacks();
  InitListening();
  websocket_hub.run();
}

void Network::SendMessageToSimulator(string msg) {
  websocket_hub.getDefaultGroup<uWS::SERVER>().broadcast(msg.data(), msg.length(), uWS::OpCode::TEXT);
}

void Network::InitCallbacks() {
  websocket_hub.onMessage([this](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
//    cout << "MESSAGE RECEIVED FROM SIMULATOR CLIENT" << endl;
    if (this->HandleSimulatorMessage != nullptr) {
      this->HandleSimulatorMessage(data, length);
    }
  });

  websocket_hub.onConnection([](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    cout << "CONNECTED TO SIMULATOR CLIENT" << endl;
  });

  websocket_hub.onDisconnection([](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
    ws.close();
    cout << "DISCONNECTED FROM SIMULATOR CLIENT" << endl;
  });
}

void Network::InitListening() {
  if (websocket_hub.listen(SIMULATOR_CLIENT_PORT)) {
    cout << "Listening to simulator on port " << SIMULATOR_CLIENT_PORT << endl;
  } else {
    cerr << "Failed to listen to simulator on port " << SIMULATOR_CLIENT_PORT << endl;
    exit(EXIT_FAILURE);
  }
}


void Network::RegisterReceiveMessageHandler(const std::function<void(char*,size_t)> ReceiveMessageHandler) {
  this->HandleSimulatorMessage = ReceiveMessageHandler;
}

}