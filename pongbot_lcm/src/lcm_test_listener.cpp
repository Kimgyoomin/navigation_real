#include <iostream>
#include <string>

#include <lcm/lcm-cpp.hpp>
#include "robotlcm/joy_cmd.hpp"

class CmdVelHandler
{
public:
  void handleMessage(
    const lcm::ReceiveBuffer * /*rbuf*/,
    const std::string & chan,
    const robotlcm::joy_cmd * msg)
  {
    std::cout << "[SUB] channel=" << chan
              << " seq=" << msg->seq
              << " utime=" << msg->utime
              << " lx=" << msg->linear_x
              << " ly=" << msg->linear_y
              << " az=" << msg->angular_z << std::endl;
  }
};

int main(int argc, char ** argv)
{
  std::string lcm_url = "udpm://239.255.76.67:7667?ttl=1";
  std::string channel = "CMD_VEL";

  if (argc >= 2) {
    lcm_url = argv[1];
  }
  if (argc >= 3) {
    channel = argv[2];
  }

  lcm::LCM lcm(lcm_url);
  if (!lcm.good()) {
    std::cerr << "Failed to initialize LCM with URL: " << lcm_url << std::endl;
    return 1;
  }

  CmdVelHandler handler;
  lcm.subscribe(channel, &CmdVelHandler::handleMessage, &handler);

  std::cout << "Listening on channel=" << channel << " url=" << lcm_url << std::endl;
  while (0 == lcm.handle()) {
  }

  return 0;
}
