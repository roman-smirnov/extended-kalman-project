#include "controller.h"
#include "network.h"

using kalman::Controller;
using kalman::Network;

int main() {
  Controller controller;
  Network network;

  controller.RegisterSendMessageHandler([&](const std::string &msg) {
    network.SendMessageToSimulator(msg);
  });

  network.RegisterReceiveMessageHandler([&](const char *data, const size_t length){
    controller.HandleSimulatorMessage(data, length);
  });

  network.StartServer();
 }