#!/usr/bin/env python3
"""Run the existing RUBI Nav2 bringup with the isolated AstarLocal parameters."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
    share = get_package_share_directory("pongbot_local_graph_insertion_planner")
    navigation_share = get_package_share_directory("pongbot_navigation")
    configured_params = RewrittenYaml(
        source_file=os.path.join(share, "config", "nav2_rubi_astar_local.yaml"),
        root_key="",
        param_rewrites={
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "default_nav_to_pose_bt_xml": os.path.join(
                share, "behavior_trees", "navigate_to_pose_astar_local_replanning.xml"),
        },
        convert_types=True)
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("autostart", default_value="true"),
        DeclareLaunchArgument("map_yaml", default_value=os.path.join(navigation_share, "maps", "RUBI_occupancy_map.yaml")),
        DeclareLaunchArgument("use_rviz", default_value="true", description="Accepted for simulation wrapper compatibility; RViz is launched separately."),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(navigation_share, "launch", "rubi_navigation.launch.py")),
            launch_arguments={
                "params_file": configured_params,
                "map_yaml": LaunchConfiguration("map_yaml"),
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "autostart": LaunchConfiguration("autostart"),
            }.items()),
    ])
