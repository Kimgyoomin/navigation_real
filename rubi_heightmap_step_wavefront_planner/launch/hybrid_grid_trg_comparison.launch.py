#!/usr/bin/env python3
"""Launch only the non-commanding Grid-vs-TRG comparison node."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory(
        'rubi_heightmap_step_wavefront_planner')
    default_parameters = os.path.join(
        package_share, 'config', 'hybrid_grid_trg_comparison_v1.yaml')
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    launch_rviz = LaunchConfiguration('launch_rviz')
    rviz_config = LaunchConfiguration('rviz_config')
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_parameters),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('launch_rviz', default_value='true'),
        DeclareLaunchArgument(
            'rviz_config', default_value=os.path.join(
                package_share, 'rviz', 'hybrid_grid_trg_comparison.rviz')),
        Node(
            package='rubi_heightmap_step_wavefront_planner',
            executable='hybrid_planner_comparison_node',
            name='rubi_hybrid_planner_comparison',
            output='screen',
            parameters=[params_file, {
                'use_sim_time': use_sim_time,
                'planner_run_mode': 'both',
            }],
        ),
        Node(
            package='rviz2', executable='rviz2', name='hybrid_comparison_rviz',
            arguments=['-d', rviz_config], condition=IfCondition(launch_rviz),
            parameters=[{'use_sim_time': use_sim_time}], output='screen',
        ),
    ])
