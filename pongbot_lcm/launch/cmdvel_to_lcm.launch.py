from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="pongbot_lcm",
            executable="cmdvel_to_lcm_node",
            name="cmdvel_to_lcm",
            output="screen",
            parameters=[{
                "cmd_vel_topic": "/cmd_vel",
                "lcm_channel": "CMD_VEL",
                "lcm_url": "udpm://239.255.76.67:7667?ttl=1",
                "publish_rate_hz": 50.0,
                "deadman_timeout_s": 0.3,
                "scale_linear_x": 1.0,
                "scale_linear_y": 1.0,
                "scale_angular_z": 1.0,
                "max_linear_x": 1.0,
                "max_linear_y": 0.75,
                "max_angular_z": 1.57,
            }]
        )
    ])
