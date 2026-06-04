#include <stdio.h>
#include <lcm/lcm-cpp.hpp>
#include "robotlcm/cmd_vel_t.hpp"

class Handler {
public:
    void handleMessage(const lcm::ReceiveBuffer*,
                       const std::string& chan,
                       const robotlcm::cmd_vel_t* msg)
    {
        // 출력 과도하게 되는거 방지하기 위해, 10개마다 한 번씩 출력
        if ((msg->seq % 10) == 0) {
            std::printf("[%s] seq=%d utime=%lld lx=%.3f ly=%.3f az=%.3f\n",
                        chan.c_str(),
                        msg->seq,
                        (long long)msg->utime,
                        msg->linear_x,
                        msg->linear_y,
                        msg->angular_z);
            std::fflush(stdout);
        }
    }
};

int main(int argc, char** argv)
{
    const std::string url="udpm://239.1.1.1:32000?ttl=0";
    lcm::LCM lcm(url);
    if(!lcm.good()) return 1;

    Handler h;
    lcm.subscribe("CMD_VEL", &Handler::handleMessage, &h);

    while(0 == lcm.handle()) {}
    return 0;

}