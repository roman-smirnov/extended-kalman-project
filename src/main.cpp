#include "controller.h"
#include "network.h"

using ekf::Controller;
using ekf::Network;

int main() {
  Controller controller;
  Network network(controller);
  controller.SetNetworkGateway(&network);
  network.StartServer();
 }