#include <stdio.h>
#include <lcm/lcm-cpp.hpp>
#include "example/example_t.hpp"

class Handler
{
    public:
        ~Handler() {}

        void handleMessage(const lcm::ReceiveBuffer* rbuf,
                const std::string& chan,
                const exlcm::example_t* msg)
        
            {
                printf("    jointstates : ");
                for (int i = 0; i < 12; i++)
                {
                    printf(" %f", msg->jointstates[i]);
                }
                printf("\n");
            }
};

int main(int argc, char** argv)
{
    lcm::LCM lcm("udpm://239.1.1.1:32000?ttl=1");
    if(!lcm.good())
        return 1;

    Handler handlerObject;
    lcm.subscribe("EXAMPLE", &Handler::handleMessage, &handlerObject);

    while(0 == lcm.handle());

    return 0;
}