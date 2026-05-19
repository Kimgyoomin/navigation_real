from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="pongbot_lcm",
            executable="joy_lcm_node",
            name="joy_to_lcm",
            output="screen",
            parameters=[{
                "joy_topic": "/joy",
                "lcm_channel": "CMD_VEL",
                "lcm_url": "udpm://239.255.76.67:7667?ttl=255",
                "publish_rate_hz": 50.0,
                "deadman_timeout_s": 0.3,
                "axis_linear_x": 1,
                "axis_linear_y": 0,
                "axis_angular_z": 3,
                "sign_linear_x": 1.0,
                "sign_linear_y": -1.0,
                "sign_angular_z": -1.0,
                "scale_linear_x": 1.0,
                "scale_linear_y": 1.0,
                "scale_angular_z": 1.0,
                "max_linear_x": 0.5,
                "max_linear_y": 0.2,
                "max_angular_z": 0.4,
                "axis_deadzone": 0.08,
                "debug_raw_joy": True,
            }]
        ),

        Node(
            package="pongbot_lcm",
            executable="cmdvel_to_lcm_node",
            name="cmdvel_to_lcm",
            output="screen",
            parameters=[{
                "cmd_vel_topic": "/cmd_vel",
                "lcm_channel": "NAV_CMD_VEL",
                "lcm_url": "udpm://239.255.76.67:7667?ttl=255",
                "publish_rate_hz": 50.0,
                "deadman_timeout_s": 0.3,
                "scale_linear_x": 1.0,
                "scale_linear_y": 1.0,
                "scale_angular_z": 1.0,
                "sign_linear_x": 1.0,
                "sign_linear_y": 1.0,
                "sign_angular_z": 1.0,
                "max_linear_x": 1.0,
                "max_linear_y": 0.75,
                "max_angular_z": 1.5,
                "debug": True,
            }]
        ),
    ])