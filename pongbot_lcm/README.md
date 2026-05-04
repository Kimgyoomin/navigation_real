# pongbot_lcm

ROS2 Humble package for:
1. Bridging `/cmd_vel` -> LCM (`robotlcm::cmd_vel_t`)
2. Testing raw LCM communication with a publisher/listener pair

## Install
```bash
sudo apt update
sudo apt install -y build-essential cmake pkg-config liblcm-dev liblcm-bin
```

## Build
Copy this package into `~/ros2_ws/src/` and then:
```bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select pongbot_lcm
source install/setup.bash
```

## Files
- `lcmtypes/cmd_vel_t.lcm`: LCM message definition
- `src/cmdvel_to_lcm_node.cpp`: ROS2 `/cmd_vel` -> LCM bridge
- `src/lcm_test_publisher.cpp`: pure LCM publisher test
- `src/lcm_test_listener.cpp`: pure LCM subscriber test

## Raw LCM communication test
Terminal 1:
```bash
ros2 run pongbot_lcm lcm_test_listener
```

Terminal 2:
```bash
ros2 run pongbot_lcm lcm_test_publisher
```

Optional args:
```bash
ros2 run pongbot_lcm lcm_test_listener -- udpm://239.255.76.67:7667?ttl=1 CMD_VEL
ros2 run pongbot_lcm lcm_test_publisher -- udpm://239.255.76.67:7667?ttl=1 CMD_VEL 20
```

## ROS2 -> LCM bridge test
Run bridge:
```bash
ros2 launch pongbot_lcm cmdvel_to_lcm.launch.py
```

Publish ROS2 cmd_vel:
```bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.2}}"
```

Observe with listener or `lcm-spy`.
