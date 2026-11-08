"""Frozen-site entry point for the new polynomial/vanilla comparison only."""
import json
from pathlib import Path
import runpy
import re
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PACKAGE = 'rubi_heightmap_step_wavefront_planner'


def start(context):
    get = lambda key: LaunchConfiguration(key).perform(context)
    run_id = get('run_id')
    if not re.fullmatch(r'[A-Za-z0-9_.-]+', run_id) or run_id in ('.', '..'):
        raise ValueError('Supply a unique simple run_id')
    if get('launch_metrics') == 'true' and (Path(get('output_dir')).expanduser()/run_id).exists():
        raise ValueError('Run output already exists; select another run_id')
    lib = Path(get_package_prefix(PACKAGE))/'lib'/PACKAGE
    helper = runpy.run_path(str(lib/'frozen_site.py'))
    manifest, _ = helper['load_site'](get('site'))
    world = Path(get('world_file')).expanduser().resolve()
    manifest['world_requested'] = dict(path=str(world), sha256=helper['digest'](world),
        verification='Requested file recorded; launch does not inspect or start Gazebo.')
    share = Path(get_package_share_directory(PACKAGE))
    args = {k: get(k) for k in ('cost_mode', 'cot_degree', 'height_weight', 'run_id', 'scenario',
        'use_sim_time', 'launch_metrics', 'launch_controller', 'launch_map_costmap', 'launch_rviz',
        'output_dir', 'planner_params', 'controller_params', 'costmap_params')}
    args.update(static_heightmap='true', input_heightmap_topic='/rubi/experiment/frozen_heightmap',
        map_yaml=manifest['assets']['occupancy']['path'],
        map_manifest_json=json.dumps(manifest, ensure_ascii=False, allow_nan=False),
        rviz_config=str(share/'rviz/grid_cost_frozen_experiment.rviz'))
    return [
        LogInfo(msg=f'Frozen site: {manifest["site_id"]}; height sha256='
            f'{manifest["height"]["sha256"]}; no online mapper is started.'),
        Node(package=PACKAGE, executable='frozen_experiment_map.py', output='screen',
             parameters=[{'site': get('site'), 'use_sim_time': get('use_sim_time') == 'true',
                 'expected_height_sha256': manifest['height']['sha256']}]),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            str(share/'launch/grid_cost_experiment.launch.py')), launch_arguments=args.items()),
    ]


def generate_launch_description():
    share = Path(get_package_share_directory(PACKAGE))
    defaults = dict(site=str(Path.home()/'legged_robot_ws/maps/sim_site_v1/site.yaml'),
        world_file=str(Path.home()/'worlds_height515_layout_clean.world'),
        cost_mode='vanilla', cot_degree='3', height_weight='5.0', run_id='',
        scenario='height515_frozen', use_sim_time='true', launch_metrics='true',
        launch_controller='true', launch_map_costmap='true', launch_rviz='true',
        output_dir=str(Path.home()/'cot_experiment/navigation_cost_comparison'),
        planner_params=str(share/'config/grid_cost_experiment.yaml'),
        controller_params=str(share/'config/grid_cost_experiment_controller.yaml'),
        costmap_params=str(share/'config/grid_cost_experiment_costmap.yaml'))
    # A unique explicit run_id makes the bag, metadata and numerical result agree.
    return LaunchDescription([DeclareLaunchArgument(k, default_value=v) if k != 'run_id'
        else DeclareLaunchArgument(k, description='Unique run id, e.g. height515_vanilla_01')
        for k, v in defaults.items()]+[OpaqueFunction(function=start)])
