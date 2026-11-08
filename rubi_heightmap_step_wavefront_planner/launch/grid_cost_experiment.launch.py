#!/usr/bin/env python3
"""Separate vanilla/CoT experiment entry point. Historical launches are unchanged."""

import os
import json
import hashlib
from datetime import datetime

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            LogInfo, OpaqueFunction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def experiment_nodes(context):
    def value(name):
        return LaunchConfiguration(name).perform(context)
    mode = value('cost_mode')
    if mode not in ('vanilla', 'cot'):
        raise ValueError('cost_mode must be vanilla or cot')
    model_path = value('cot_model_file')
    with open(model_path, 'rb') as f:
        model_bytes = f.read()
    model = json.loads(model_bytes)
    degree = int(value('cot_degree'))
    chosen = next(m for m in model['models'] if m['degree'] == degree)
    if model['domain_cm'] != [0, 11] or model['constraint'] != 'f(0)=0':
        raise ValueError('Expected the Flat–11 cm origin-constrained model')
    sim = value('use_sim_time').lower() == 'true'
    overrides = {
        'use_sim_time': sim, 'planner_run_mode': 'grid_only',
        'experiment.publish_metrics': True, 'experiment.run_id': value('run_id'),
        'experiment.model_sha256': hashlib.sha256(model_bytes).hexdigest(),
        'evaluation.grid_cot_height_cost_enabled': mode == 'cot',
        'evaluation.grid_cot_polynomial_coefficients_cm': chosen['coefficients_descending_cm'],
        'evaluation.grid_cot_flat_baseline': model['baseline_CoT'],
        'evaluation.grid_cot_calibration_distance_m': model['calibration_distance_m'],
        'evaluation.grid_cot_max_height_m': model['domain_cm'][1] / 100.0,
        'evaluation.grid_comparison_max_event_height_m': model['domain_cm'][1] / 100.0,
        'evaluation.height_cost_weight': float(value('height_weight')),
    }
    if value('input_heightmap_topic'):
        overrides['input_heightmap_topic'] = value('input_heightmap_topic')
    overrides['static_heightmap'] = value('static_heightmap').lower() == 'true'
    nodes = [Node(package='rubi_heightmap_step_wavefront_planner',
        executable='hybrid_planner_comparison_node', name='rubi_hybrid_planner_comparison',
        output='screen', parameters=[value('planner_params'), overrides])]
    if value('launch_metrics').lower() == 'true':
        nodes.append(Node(package='rubi_heightmap_step_wavefront_planner',
            executable='navigation_experiment_metrics.py', output='screen', parameters=[{
                'use_sim_time': sim, 'run_id': value('run_id'), 'cost_mode': mode,
                'scenario': value('scenario'), 'output_dir': value('output_dir'),
                'mass_kg': model['mass_kg'], 'gravity_m_s2': model['gravity_m_s2'],
                'joint_topic': value('joint_topic'), 'odom_topic': value('odom_topic'),
                'map_manifest_json': ParameterValue(value('map_manifest_json'), value_type=str),
            }]))
    return nodes


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
        DeclareLaunchArgument('cost_mode', default_value='vanilla', choices=['vanilla', 'cot']),
        DeclareLaunchArgument('cot_degree', default_value='3', choices=['1', '2', '3']),
        DeclareLaunchArgument('height_weight', default_value='5.0',
                              description='Same weight for both modes; inherited vanilla value'),
        DeclareLaunchArgument('cot_model_file', default_value=os.path.join(
            share, 'config', 'cot_fit_240trials_20261019.json')),
        DeclareLaunchArgument('run_id', default_value=datetime.now().strftime('run_%Y%m%d_%H%M%S')),
        DeclareLaunchArgument('scenario', default_value='unspecified'),
        DeclareLaunchArgument('static_heightmap', default_value='false'),
        DeclareLaunchArgument('input_heightmap_topic', default_value=''),
        DeclareLaunchArgument('map_manifest_json', default_value='{}'),
        DeclareLaunchArgument('output_dir', default_value=os.path.expanduser(
            '~/cot_experiment/navigation_cost_comparison')),
        DeclareLaunchArgument('launch_metrics', default_value='true'),
        DeclareLaunchArgument('joint_topic', default_value='/rubi_gazebo_legacy/joint_states_500hz'),
        DeclareLaunchArgument('odom_topic', default_value='/ground_truth/odom'),
        DeclareLaunchArgument('planner_params' , default_value=os.path.join(
            share, 'config', 'grid_cost_experiment.yaml')),
        DeclareLaunchArgument('controller_params', default_value=os.path.join(
            share, 'config', 'grid_cost_experiment_controller.yaml')),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('launch_map_costmap', default_value='true'),
        DeclareLaunchArgument('launch_controller', default_value='true'),
        DeclareLaunchArgument('map_yaml', default_value=os.path.join(
            share, 'maps', 'RUBI_occupancy_map.yaml')),
        DeclareLaunchArgument('costmap_params', default_value=os.path.join(
            share, 'config', 'grid_cost_experiment_costmap.yaml')),
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
        OpaqueFunction(function=experiment_nodes),
        Node(package='pongbot_navigation',
             executable='simple_pure_pursuit_controller',
             name='simple_pure_pursuit_controller', output='screen',
             condition=IfCondition(launch_controller),
             parameters=[controller_params, {'use_sim_time': use_sim_time,
                                               'path_topic': '/rubi/planner_comparison/grid/tracking_path'}]),
        Node(package='rviz2', executable='rviz2', name='hybrid_grid_rviz',
             arguments=['-d', rviz_config], condition=IfCondition(launch_rviz),
             parameters=[{'use_sim_time': use_sim_time}], output='screen'),
    ])
