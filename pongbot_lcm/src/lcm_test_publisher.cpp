#include <chrono>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <string>
#include <thread>

#include <lcm/lcm-cpp.hpp>
#include "robotlcm/joy_cmd.hpp"

int main(int argc, char ** argv)
{
  std::string lcm_url = "udpm://239.255.76.67:7667?ttl=1";
  std::string channel = "CMD_VEL";
  double rate_hz = 20.0;

  if (argc >= 2) {
    lcm_url = argv[1];
  }
  if (argc >= 3) {
    channel = argv[2];
  }
  if (argc >= 4) {
    rate_hz = std::stod(argv[3]);
  }

  lcm::LCM lcm(lcm_url);
  if (!lcm.good()) {
    std::cerr << "Failed to initialize LCM with URL: " << lcm_url << std::endl;
    return 1;
  }

  std::cout << "Publishing test cmd_vel on channel=" << channel
            << " url=" << lcm_url << " rate=" << rate_hz << " Hz" << std::endl;

  int32_t seq = 0;
  const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, rate_hz));
  const auto t0 = std::chrono::steady_clock::now();

  while (true) {
    const auto now = std::chrono::steady_clock::now();
    const double t = std::chrono::duration<double>(now - t0).count();

    robotlcm::joy_cmd msg{};
    msg.utime = static_cast<int64_t>(
      std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count());
    msg.seq = seq++;

    // Easy-to-recognize test pattern
    msg.linear_x = static_cast<float>(0.2 * std::sin(0.5 * t));
    msg.linear_y = static_cast<float>(0.1 * std::cos(0.25 * t));
    msg.angular_z = static_cast<float>(0.3 * std::sin(0.8 * t));

    lcm.publish(channel, &msg);

    std::cout << "[PUB] seq=" << msg.seq
              << " lx=" << msg.linear_x
              << " ly=" << msg.linear_y
              << " az=" << msg.angular_z << std::endl;

    std::this_thread::sleep_for(period);
  }

  return 0;
}
