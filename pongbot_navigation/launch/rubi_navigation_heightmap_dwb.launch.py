#!/usr/bin/env python3
# flake8: noqa: Q000
"""Bring up Nav2 DWB with the FastDEM Wavefront global-planner overlay."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def _configured(source, use_sim_time):
    return ParameterFile(
        RewrittenYaml(
            source_file=source,
            root_key="",
            param_rewrites={"use_sim_time": use_sim_time},
            convert_types=True,
        ),
        allow_substs=True,
    )


def generate_launch_description():
    package_share = get_package_share_directory("pongbot_navigation")
    map_yaml = LaunchConfiguration("map_yaml")
    base_params_file = LaunchConfiguration("base_params_file")
    heightmap_overlay_file = LaunchConfiguration("heightmap_overlay_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")

    arguments = [
        DeclareLaunchArgument(
            "map_yaml",
            default_value=os.path.join(
                package_share, "maps", "RUBI_occupancy_map.yaml"),
        ),
        DeclareLaunchArgument(
            "base_params_file",
            default_value=os.path.join(
                package_share, "config", "nav2_rubi_dwb.yaml"),
        ),
        DeclareLaunchArgument(
            "heightmap_overlay_file",
            default_value=os.path.join(
                package_share,
                "config",
                "nav2_rubi_heightmap_wavefront_overlay.yaml",
            ),
        ),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("autostart", default_value="true"),
    ]

    configured_base_params = _configured(base_params_file, use_sim_time)
    configured_heightmap_overlay = _configured(
        heightmap_overlay_file, use_sim_time)

    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "yaml_filename": map_yaml,
            "topic_name": "map",
            "frame_id": "map",
        }],
    )

    planner_server = Node(
        package="nav2_planner",
        executable="planner_server",
        name="planner_server",
        output="screen",
        parameters=[configured_base_params, configured_heightmap_overlay],
    )
    controller_server = Node(
        package="nav2_controller",
        executable="controller_server",
        name="controller_server",
        output="screen",
        parameters=[configured_base_params],
    )
    smoother_server = Node(
        package="nav2_smoother",
        executable="smoother_server",
        name="smoother_server",
        output="screen",
        parameters=[configured_base_params],
    )
    behavior_server = Node(
        package="nav2_behaviors",
        executable="behavior_server",
        name="behavior_server",
        output="screen",
        parameters=[configured_base_params],
    )
    bt_navigator = Node(
        package="nav2_bt_navigator",
        executable="bt_navigator",
        name="bt_navigator",
        output="screen",
        parameters=[configured_base_params, configured_heightmap_overlay],
    )

    map_lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_map",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "autostart": autostart,
            "node_names": ["map_server"],
        }],
    )
    navigation_lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "autostart": autostart,
            "node_names": [
                "controller_server",
                "smoother_server",
                "planner_server",
                "behavior_server",
                "bt_navigator",
            ],
        }],
    )

    return LaunchDescription(arguments + [
        map_server,
        planner_server,
        controller_server,
        smoother_server,
        behavior_server,
        bt_navigator,
        map_lifecycle_manager,
        navigation_lifecycle_manager,
    ])
