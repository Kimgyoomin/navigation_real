#!/usr/bin/env python3
"""Launch a non-commanding Grid A* CoT soft-cost experiment."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_share = get_package_share_directory(
        'rubi_heightmap_step_wavefront_planner')
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    launch_rviz = LaunchConfiguration('launch_rviz')
    launch_map_costmap = LaunchConfiguration('launch_map_costmap')
    map_yaml = LaunchConfiguration('map_yaml')
    costmap_params = LaunchConfiguration('costmap_params')
    rviz_config = LaunchConfiguration('rviz_config')
    distance_weight = ParameterValue(
        LaunchConfiguration('distance_weight'), value_type=float)
    height_cost_weight = ParameterValue(
        LaunchConfiguration('height_cost_weight'), value_type=float)
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=os.path.join(
            package_share, 'config', 'hybrid_grid_cot_cost.yaml')),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('launch_map_costmap', default_value='true'),
        DeclareLaunchArgument('map_yaml', default_value=os.path.join(
            package_share, 'maps', 'RUBI_occupancy_map.yaml')),
        DeclareLaunchArgument('costmap_params', default_value=os.path.join(
            package_share, 'config', 'hybrid_global_costmap.yaml')),
        DeclareLaunchArgument('launch_rviz', default_value='true'),
        DeclareLaunchArgument('rviz_config', default_value=os.path.join(
            package_share, 'rviz', 'hybrid_grid_trg_comparison.rviz')),
        DeclareLaunchArgument('distance_weight', default_value='1.0'),
        DeclareLaunchArgument('height_cost_weight', default_value='1.0'),
        LogInfo(msg='Grid CoT mode launches no controller and no /cmd_vel publisher.'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                package_share, 'launch', 'hybrid_costmap_bringup.launch.py')),
            condition=IfCondition(launch_map_costmap),
            launch_arguments={
                'map_yaml': map_yaml,
                'costmap_params': costmap_params,
                'use_sim_time': use_sim_time,
                'autostart': 'true',
            }.items()),
        Node(
            package='rubi_heightmap_step_wavefront_planner',
            executable='hybrid_planner_comparison_node',
            name='rubi_hybrid_planner_comparison',
            output='screen',
            parameters=[params_file, {
                'use_sim_time': use_sim_time,
                'planner_run_mode': 'grid_only',
                'replanning.enabled': False,
                'evaluation.distance_weight': distance_weight,
                'evaluation.height_cost_weight': height_cost_weight,
            }]),
        Node(
            package='rviz2', executable='rviz2', name='hybrid_grid_cot_rviz',
            arguments=['-d', rviz_config], condition=IfCondition(launch_rviz),
            parameters=[{'use_sim_time': use_sim_time}], output='screen'),
    ])
