#include <cstdio>
#include <chrono>
#include <thread>

#include <lcm/lcm-cpp.hpp>
#include "robotlcm/robot_data_t.hpp"

int main(int argc, char ** argv)
{
    // 1) 단일 PC에서 돌리는거라면, ttl = 0 + iface = lo 로 "네트워크 문제 제거"
    // 2) 이기종 PC 사이 네트워크 설정 위해 ttl = 1 + iface = 실제 NIC(enp89s0 등)로 변경
    lcm::LCM lcm("udpm://239.1.1.1:32000?ttl=1");
    if(!lcm.good()) {
        std::printf("LCM init error\n");
        return 1;
    }

    robotlcm::robot_data_t robot_data{};        // 0 초기화

    for(int i = 0; i < 12; i++) {
        robot_data.jointstates[i]   = static_cast<float>(i);
        robot_data.statusword[i]    = 0.0f;
    }

    robot_data.gps[0]   = 37.0f;
    robot_data.gps[1]   = 127.0f;
    robot_data.gps[2]   = 0.0f;

    const auto period = std::chrono::milliseconds(10);      // 100Hz

    while (true){
        // 업데이트가 잘 되는지 확인!!!!
        robot_data.jointstates[0] += 0.01f;

        static int cnt = 0;

        // check before publish
        cnt++;
        if (cnt % 100 == 0) {
            printf("[SERVER] publishing jointstates[0]=%f\n", robot_data.jointstates[0]);
        }

        
        lcm.publish("data_robot", &robot_data);
        std::this_thread::sleep_for(period);
    }

    return 0;
}