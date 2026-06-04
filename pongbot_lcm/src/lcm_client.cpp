#include <stdio.h>
#include <lcm/lcm-cpp.hpp>
#include "robotlcm/robot_data_t.hpp"

class Handler 
{
public:
    ~Handler() {}

    void handleMessage(const lcm::ReceiveBuffer* rbuf,
                       const std::string& chan, 
                       const robotlcm::robot_data_t* msg)
    {
        printf("[%s] jointstates :", chan.c_str());
        for(int i = 0; i < 12; i++)
            printf(" %f", msg->jointstates[i]);
        printf("\n");
    }
};

int main(int argc, char** argv)
{
    lcm::LCM lcm("udpm://239.1.1.1:32000?ttl=0&iface=enp89s0");
    if(!lcm.good())
        return 1;

    Handler handlerObject;
    lcm.subscribe("data_robot", &Handler::handleMessage, &handlerObject);

    // handle()은 메시지 받을 때까지 블록됨
    // 0 == handle() 이면 정상
    while(0 == lcm.handle());

    return 0;
}
