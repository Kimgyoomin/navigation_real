#!/usr/bin/env python3
"""Launch only the standalone RUBI Simple Pure Pursuit controller."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    package_share = get_package_share_directory("pongbot_navigation")
    params_file = LaunchConfiguration("params_file")
    use_sim_time = LaunchConfiguration("use_sim_time")

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key="",
            param_rewrites={"use_sim_time": use_sim_time},
            convert_types=True,
        ),
        allow_substs=True,
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "params_file",
            default_value=os.path.join(
                package_share, "config", "simple_pure_pursuit.yaml"),
        ),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        Node(
            package="pongbot_navigation",
            executable="simple_pure_pursuit_controller",
            name="simple_pure_pursuit_controller",
            output="screen",
            parameters=[configured_params],
        ),
    ])
