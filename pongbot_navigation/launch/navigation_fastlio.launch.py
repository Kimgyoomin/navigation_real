#!/usr/bin/env python3
"""
navigation_fastlio.launch.py

FAST-LIO localization 기반 Nav2-only launch file.

Assumed FAST-LIO TF tree:
  map
   └── camera_init          # REP-105 odom-equivalent local frame
        └── body            # FAST-LIO LiDAR-IMU body frame
             └── base_link  # ROS/Nav2 robot center frame, FLU
                  └── livox_frame

Important:
  This launch file does NOT publish static TF.
  FAST-LIO localization launch must publish:
    body -> base_link
    base_link -> livox_frame

This launch only starts:
  map_server
  planner_server
  controller_server
  smoother_server
  behavior_server
  bt_navigator
  lifecycle_manager_navigation
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    params_file = LaunchConfiguration("params_file")
    map_yaml = LaunchConfiguration("map_yaml")
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")

    declared_arguments = [
        DeclareLaunchArgument(
            "params_file",
            default_value="/home/ams4976/ros2_ws/src/pongbot_navigation/config/nav2_rubi_fastlio_point.yaml",
            description="Nav2 params for FAST-LIO localization mode.",
        ),
        DeclareLaunchArgument(
            "map_yaml",
            default_value="/home/ams4976/ros2_ws/src/pongbot_navigation/maps/building_1f_map_nav2_yaw0.yaml",
            description="2D occupancy map used by Nav2 static layer.",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation time.",
        ),
        DeclareLaunchArgument(
            "autostart",
            default_value="true",
            description="Automatically activate lifecycle nodes.",
        ),
    ]

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key="",
            param_rewrites={
                "use_sim_time": use_sim_time,
            },
            convert_types=True,
        ),
        allow_substs=True,
    )

    # -------------------------------------------------------------------------
    # Nav2 map server
    #
    # This is not localization. It publishes the 2D OccupancyGrid /map
    # required by the Nav2 global costmap static layer.
    # -------------------------------------------------------------------------
    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "yaml_filename": map_yaml,
            }
        ],
    )

    planner_server = Node(
        package="nav2_planner",
        executable="planner_server",
        name="planner_server",
        output="screen",
        parameters=[configured_params],
    )

    controller_server = Node(
        package="nav2_controller",
        executable="controller_server",
        name="controller_server",
        output="screen",
        parameters=[configured_params],
    )

    smoother_server = Node(
        package="nav2_smoother",
        executable="smoother_server",
        name="smoother_server",
        output="screen",
        parameters=[configured_params],
    )

    behavior_server = Node(
        package="nav2_behaviors",
        executable="behavior_server",
        name="behavior_server",
        output="screen",
        parameters=[configured_params],
    )

    bt_navigator = Node(
        package="nav2_bt_navigator",
        executable="bt_navigator",
        name="bt_navigator",
        output="screen",
        parameters=[configured_params],
    )

    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "autostart": autostart,
                "node_names": [
                    "map_server",
                    "planner_server",
                    "controller_server",
                    "smoother_server",
                    "behavior_server",
                    "bt_navigator",
                ],
            }
        ],
    )

    return LaunchDescription(
        declared_arguments
        + [
            map_server,
            planner_server,
            controller_server,
            smoother_server,
            behavior_server,
            bt_navigator,
            lifecycle_manager,
        ]
    )