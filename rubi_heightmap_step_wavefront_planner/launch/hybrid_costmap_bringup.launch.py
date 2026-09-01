#!/usr/bin/env python3
"""Bring up the installed RUBI map and a static+inflation global costmap."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('rubi_heightmap_step_wavefront_planner')
    map_yaml = LaunchConfiguration('map_yaml')
    costmap_params = LaunchConfiguration('costmap_params')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')

    return LaunchDescription([
        DeclareLaunchArgument(
            'map_yaml', default_value=os.path.join(
                share, 'maps', 'RUBI_occupancy_map.yaml')),
        DeclareLaunchArgument(
            'costmap_params', default_value=os.path.join(
                share, 'config', 'hybrid_global_costmap.yaml')),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('autostart', default_value='true'),
        Node(
            package='nav2_map_server', executable='map_server',
            name='map_server', output='screen',
            parameters=[{
                'yaml_filename': map_yaml,
                'use_sim_time': use_sim_time,
            }]),
        Node(
            package='rubi_heightmap_step_wavefront_planner',
            executable='hybrid_global_costmap_node',
            namespace='global_costmap', name='global_costmap', output='screen',
            parameters=[costmap_params, {'use_sim_time': use_sim_time}]),
        Node(
            package='nav2_lifecycle_manager', executable='lifecycle_manager',
            name='lifecycle_manager_hybrid_costmap', output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': autostart,
                # Standalone Costmap2DROS has no lifecycle-manager bond in
                # Humble. Disable bond monitoring while retaining transitions.
                'bond_timeout': 0.0,
                'node_names': [
                    'map_server',
                    'global_costmap/global_costmap',
                ],
            }]),
    ])
