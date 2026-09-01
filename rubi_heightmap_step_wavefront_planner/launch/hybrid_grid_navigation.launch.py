#!/usr/bin/env python3
"""Run Grid A* with exactly one shared Simple Pure Pursuit controller."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            LogInfo)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('rubi_heightmap_step_wavefront_planner')
    planner_params = LaunchConfiguration('planner_params')
    controller_params = LaunchConfiguration('controller_params')
    use_sim_time = LaunchConfiguration('use_sim_time')
    launch_rviz = LaunchConfiguration('launch_rviz')
    launch_map_costmap = LaunchConfiguration('launch_map_costmap')
    launch_controller = LaunchConfiguration('launch_controller')
    map_yaml = LaunchConfiguration('map_yaml')
    costmap_params = LaunchConfiguration('costmap_params')
    rviz_config = LaunchConfiguration('rviz_config')
    return LaunchDescription([
        DeclareLaunchArgument('planner_params', default_value=os.path.join(
            share, 'config', 'hybrid_grid_trg_comparison_v1.yaml')),
        DeclareLaunchArgument('controller_params', default_value=os.path.join(
            share, 'config', 'hybrid_navigation_controller.yaml')),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('launch_map_costmap', default_value='true'),
        DeclareLaunchArgument('launch_controller', default_value='true'),
        DeclareLaunchArgument('map_yaml', default_value=os.path.join(
            share, 'maps', 'RUBI_occupancy_map.yaml')),
        DeclareLaunchArgument('costmap_params', default_value=os.path.join(
            share, 'config', 'hybrid_global_costmap.yaml')),
        DeclareLaunchArgument('launch_rviz', default_value='true'),
        DeclareLaunchArgument('rviz_config', default_value=os.path.join(
            share, 'rviz', 'hybrid_grid_trg_comparison.rviz')),
        LogInfo(msg='\n[HYBRID NAV SAFETY]\nmode=grid_only\n'
                    'planner_cmd_vel_publishers=0\n'
                    'controller_expected_cmd_vel_topic=/cmd_vel\n'
                    'Do not run rubi_navigation_dwb.launch.py with this launch.\n'
                    'Do not run another controller on /cmd_vel simultaneously.'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                share, 'launch', 'hybrid_costmap_bringup.launch.py')),
            condition=IfCondition(launch_map_costmap),
            launch_arguments={
                'map_yaml': map_yaml,
                'costmap_params': costmap_params,
                'use_sim_time': use_sim_time,
                'autostart': 'true',
            }.items()),
        Node(package='rubi_heightmap_step_wavefront_planner',
             executable='hybrid_planner_comparison_node',
             name='rubi_hybrid_planner_comparison', output='screen',
             parameters=[planner_params, {'use_sim_time': use_sim_time,
                                           'planner_run_mode': 'grid_only'}]),
        Node(package='pongbot_navigation',
             executable='simple_pure_pursuit_controller',
             name='simple_pure_pursuit_controller', output='screen',
             condition=IfCondition(launch_controller),
             parameters=[controller_params, {'use_sim_time': use_sim_time,
                                               'path_topic': '/rubi/planner_comparison/grid/path'}]),
        Node(package='rviz2', executable='rviz2', name='hybrid_grid_rviz',
             arguments=['-d', rviz_config], condition=IfCondition(launch_rviz),
             parameters=[{'use_sim_time': use_sim_time}], output='screen'),
    ])
