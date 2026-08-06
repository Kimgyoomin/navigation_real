from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="pongbot_lcm",
            executable="joy_lcm_node",   # CMake target name과 반드시 일치해야 함
            name="joy_to_lcm",
            output="screen",
            parameters=[{
                "joy_topic": "/joy",
                "cmd_vel_topic": "/cmd_vel",
                "lcm_channel": "CMD_VEL",
                # LCM is confined to the directly connected robot LAN.
                "lcm_url": "udpm://239.255.76.67:7667?ttl=1",

                "publish_rate_hz": 50.0,
                "deadman_timeout_s": 0.3,

                # Asus ROG Ally X 기준 축 매핑
                "axis_linear_x": 1,
                "axis_linear_y": 0,
                "axis_angular_z": 3,

                # 최종 부호
                "sign_linear_x": 1.0,
                "sign_linear_y": -1.0,
                "sign_angular_z": -1.0,

                # 추가 배율
                "scale_linear_x": 1.0,
                "scale_linear_y": 1.0,
                "scale_angular_z": 1.0,

                # 최대 명령값
                "max_linear_x": 0.5,
                "max_linear_y": 0.2,
                "max_angular_z": 0.4,

                # deadzone
                "axis_deadzone": 0.08,

                # 디버그 출력
                "debug_raw_joy": True,
            }]
        )
    ])
