//* Header file
#include <iostream>

#include "Utilities/Utilities.h"


namespace pongbot_heightmap_cupy {

std::string getLCMUrl(int64_t ttl)
{
    assert(ttl >= 0 && ttl <= 255);
    return "udpm://239.255.76.67:7667?ttl=" + std::to_string(ttl);
}

}