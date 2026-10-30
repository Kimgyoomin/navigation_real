#!/usr/bin/env python3
"""RUBI 2D OccupancyGrid map server and Nav2 bringup.

FAST-LIO localization is intentionally not launched here. Start
localization_nav_rubi.launch.py separately before this launch file.
"""

import os
import math
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml
from navigation_dds.profiles import dds_argument, dds_group


def generate_launch_description():
    package_share = get_package_share_directory("pongbot_navigation")

    default_map_yaml = os.path.join(
        package_share,
        "maps",
        "RUBI_occupancy_map.yaml",
    )
    default_params_file = os.path.join(
        package_share,
        "config",
        "nav2_rubi_dwb.yaml",
    )

    map_yaml = LaunchConfiguration("map_yaml")
    params_file = LaunchConfiguration("params_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    use_fastlio_velocity = LaunchConfiguration("use_fastlio_velocity")

    declared_arguments = [
        dds_argument(),
        DeclareLaunchArgument(
            "use_fastlio_velocity",
            default_value="false",
            choices=["true", "false"],
            description="Opt-in FAST-LIO base velocity + DWB LimitedAccelGenerator (offline validated; requires live stopped verification)",
        ),
        DeclareLaunchArgument(
            "map_yaml",
            default_value="/home/rclab/ros2_ws/src/navigation_real/FAST_LIO_LOCALIZATION2/maps/RUBI_1F_occupancy_map.yaml",
            description="Nav2 OccupancyGrid map YAML file",
        ),
        DeclareLaunchArgument(
            "params_file",
            default_value=default_params_file,
            description="RUBI Nav2 parameter file",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use Gazebo /clock when true",
        ),
        DeclareLaunchArgument(
            "autostart",
            default_value="true",
            description="Automatically activate Nav2 lifecycle nodes",
        ),
    ]

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key="",
            param_rewrites={"use_sim_time": use_sim_time},
            convert_types=True,
        ),
        allow_substs=True,
    )

    # Both costmaps share one lightweight filter. FAST-LIO keeps its raw input.
    costmap_sector_filter = Node(
        package="pongbot_navigation",
        executable="costmap_sector_filter",
        name="costmap_sector_filter",
        output="screen",
        parameters=[configured_params],
    )

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
        parameters=[configured_params],
    )

    # Keep the baseline params_file intact when the experiment is disabled.
    # The controller reads twist directly; it does not transform its child frame.
    def nav2_consumers(context):
        controller_overrides = {}
        bt_overrides = {}
        if use_fastlio_velocity.perform(context) == "true":
            with open(params_file.perform(context), encoding="utf-8") as stream:
                params = yaml.safe_load(stream)
            hz = float(params["controller_server"]["ros__parameters"]["controller_frequency"])
            if not math.isfinite(hz) or hz <= 0:
                raise ValueError("Velocity feedback requires a positive controller_frequency")
            controller_overrides = {
                "odom_topic": "/fastlio/odometry_base",
                "FollowPath.trajectory_generator_name": "dwb_plugins::LimitedAccelGenerator",
                "FollowPath.sim_period": 1.0 / hz,
            }
            bt_overrides = {"odom_topic": "/fastlio/odometry_base"}
        return [
            Node(package="nav2_controller", executable="controller_server",
                 name="controller_server", output="screen",
                 parameters=[configured_params, controller_overrides]),
            Node(package="nav2_bt_navigator", executable="bt_navigator",
                 name="bt_navigator", output="screen",
                 parameters=[configured_params, bt_overrides]),
        ]

    velocity_feedback = Node(
        package="pongbot_navigation", executable="fastlio_velocity_feedback",
        name="fastlio_velocity_feedback", output="screen",
        condition=IfCondition(use_fastlio_velocity),
        parameters=[{"use_sim_time": use_sim_time}],
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

    return LaunchDescription(
        declared_arguments
        + [dds_group([
            costmap_sector_filter,
            map_server,
            planner_server,
            velocity_feedback,
            OpaqueFunction(function=nav2_consumers),
            smoother_server,
            behavior_server,
            map_lifecycle_manager,
            navigation_lifecycle_manager,
        ], 'RUBI navigation')]
    )
