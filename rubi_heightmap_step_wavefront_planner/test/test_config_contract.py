from pathlib import Path
import subprocess

import yaml


CONFIG = Path(__file__).resolve().parents[1] / 'config' / 'step_wavefront_v0.yaml'


def _parameters(path):
    with path.open(encoding='utf-8') as stream:
        document = yaml.safe_load(stream)
    node = document.get('rubi_heightmap_step_wavefront_planner')
    if node is None:
        node = document.get('rubi_hybrid_planner_comparison')
    if node is None:
        node = next(iter(document.values()))
    return node['ros__parameters']


def test_step_wavefront_v0_contract():
    with CONFIG.open(encoding='utf-8') as stream:
        parameters = yaml.safe_load(stream)[
            'rubi_heightmap_step_wavefront_planner'
        ]['ros__parameters']
    expected = {
        'base_frame': 'base_link',
        'map_resolution_m': 0.05,
        'lattice_tolerance_m': 0.01,
        'max_grid_cells': 5000000,
        'transform_timeout_s': 0.20,
        'snap_start_to_valid_map': True,
        'snap_goal_to_valid_map': True,
        'start_snap_radius_m': 0.30,
        'goal_snap_radius_m': 0.25,
        'hard_clearance_radius_m': 0.20,
        'edge_check_spacing_m': 0.025,
        'max_crossable_height_jump_m': 0.08,
        'height_noise_floor_m': 0.01,
        'height_cost_exponent': 2.0,
        'distance_weight': 1.0,
        'height_cost_weight': 5.0,
        'node_sampling_distance_m': 0.30,
        'samples_per_expansion': 20,
        'merge_radius_m': 0.20,
        'neighbor_connection_radius_m': 0.45,
        'goal_connection_distance_m': 0.45,
        'max_nodes': 4000,
        'max_expansions': 4000,
        'max_graph_build_time_ms': 5000,
        'post_goal_expansions': 50,
        'path_output_spacing_m': 0.05,
        'path_invalid_confirmations': 2,
        'path_recovery_confirmations': 2,
        'max_replan_attempts': 5,
        'replan_retry_period_s': 0.5,
        'replan_retry_requires_new_map': True,
        'max_rejected_markers': 5000,
    }
    for name, value in expected.items():
        assert parameters[name] == value


def test_offline_profile_enables_bounded_snapping_without_local_paths():
    offline_path = CONFIG.parent / 'step_wavefront_offline.yaml'
    parameters = _parameters(offline_path)
    assert parameters['input_cloud_topic'] == '/rubi/offline_heightmap'
    assert parameters['snap_start_to_valid_map'] is True
    assert parameters['snap_goal_to_valid_map'] is True
    assert parameters['start_snap_radius_m'] == 0.30
    assert parameters['goal_snap_radius_m'] == 0.25
    assert parameters['replan_retry_requires_new_map'] is False
    assert '/home/' not in offline_path.read_text(encoding='utf-8')


def test_hybrid_comparison_profile_is_isolated_and_reproducible():
    path = CONFIG.parent / 'hybrid_grid_trg_comparison_v0.yaml'
    parameters = _parameters(path)
    assert parameters['input_costmap_topic'] == '/global_costmap/costmap_raw'
    assert parameters['input_heightmap_topic'] == '/fastdem/mapping/cloud_global'
    assert parameters['comparison_goal_topic'] == '/rubi/planner_comparison/goal'
    assert parameters['costmap_unknown_is_blocked'] is True
    assert parameters['sampling_policy'] == 'trg_random_ring'
    assert parameters['sampling_random_seed'] == 42
    assert parameters['max_sampling_trials_per_expansion'] == 1000
    assert parameters['max_crossable_height_jump_m'] == 0.08
    assert parameters['planner_run_mode'] == 'both'
    assert parameters['grid_visualization_max_cells'] == 10000
    assert '/cmd_vel' not in path.read_text(encoding='utf-8')

    launch_path = CONFIG.parents[1] / 'launch' / 'hybrid_grid_trg_comparison.launch.py'
    launch_text = launch_path.read_text(encoding='utf-8')
    assert 'hybrid_planner_comparison_node' in launch_text
    assert 'pongbot_navigation' not in launch_text
    assert 'fastdem' not in launch_text.lower()
    assert 'simple_pure_pursuit_controller' not in launch_text


def test_v3_original_trg_profile_contract():
    path = CONFIG.parent / 'hybrid_grid_trg_comparison_v1.yaml'
    parameters = _parameters(path)
    expected = {
        'sampling.policy': 'original_trg_random_ring',
        'sampling.trg_expand_distance_m': 0.30,
        'sampling.trg_robot_size_m': 0.20,
        'sampling.trg_sample_num': 20,
        'sampling.trg_max_trial_samples': 1000,
        'sampling.trg_height_threshold_m': 0.08,
        'sampling.trg_collision_threshold': 0.10,
        'sampling.trg_random_seed': 42,
        'sampling.trg_randomize_seed': False,
        'sampling.trg_neighbor_connection_radius_m': 0.30,
        'evaluation.max_crossable_height_jump_m': 0.08,
    }
    for name, value in expected.items():
        assert parameters[name] == value
    root = CONFIG.parents[1]
    for launch_name in (
            'hybrid_grid_navigation.launch.py',
            'hybrid_sampling_navigation.launch.py',
            'hybrid_grid_trg_comparison.launch.py'):
        text = (root / 'launch' / launch_name).read_text(encoding='utf-8')
        assert 'hybrid_grid_trg_comparison_v1.yaml' in text


def test_hybrid_navigation_launches_use_one_identical_controller_profile():
    root = CONFIG.parents[1]
    controller = _parameters(root / 'config' / 'hybrid_navigation_controller.yaml')
    expected = {
        'control_frequency_hz': 20.0,
        'lookahead_distance_m': 0.35,
        'nominal_linear_velocity_mps': 0.55,
        'min_tracking_velocity_mps': 0.25,
        'max_linear_velocity_mps': 0.70,
        'max_angular_velocity_rps': 1.50,
        'curvature_velocity_gain': 1.0,
        'max_linear_acceleration_mps2': 7.0,
        'max_angular_acceleration_rps2': 7.0,
        'goal_tolerance_m': 0.20,
        'max_cross_track_error_m': 0.40,
    }
    for name, value in expected.items():
        assert controller[name] == value
    for filename, mode, topic in (
            ('hybrid_grid_navigation.launch.py', 'grid_only',
             '/rubi/planner_comparison/grid/path'),
            ('hybrid_sampling_navigation.launch.py', 'sampling_only',
             '/rubi/planner_comparison/sampling/path')):
        text = (root / 'launch' / filename).read_text(encoding='utf-8')
        assert text.count("executable='simple_pure_pursuit_controller'") == 1
        assert f"'planner_run_mode': '{mode}'" in text
        assert f"'path_topic': '{topic}'" in text
        assert 'Do not run another controller on /cmd_vel simultaneously.' in text


def test_invalid_planner_run_mode_rejected_at_startup():
    completed = subprocess.run([
        'ros2', 'run', 'rubi_heightmap_step_wavefront_planner',
        'hybrid_planner_comparison_node', '--ros-args',
        '-p', 'planner_run_mode:=invalid'],
        check=False, capture_output=True, text=True, timeout=10)
    assert completed.returncode != 0
    assert 'planner_run_mode must be' in completed.stderr


def test_rviz_has_only_top_level_supported_displays():
    rviz = CONFIG.parents[1] / 'rviz' / 'step_wavefront_v0.rviz'
    with rviz.open(encoding='utf-8') as stream:
        document = yaml.safe_load(stream)
    classes = [item['Class'] for item in document['Visualization Manager']['Displays']]
    assert 'rviz_default_plugins/Group' not in classes
    assert 'rviz_default_plugins/PointCloud2' in classes
    assert 'rviz_default_plugins/Path' in classes
    assert classes.count('rviz_default_plugins/MarkerArray') == 4
    assert classes.count('rviz_default_plugins/Marker') == 1
    panels = [item['Class'] for item in document['Panels']]
    assert panels == [
        'rviz_common/Displays',
        'rviz_common/Selection',
        'rviz_common/Tool Properties',
    ]
    topics = {
        item['Topic']['Value'] for item in document['Visualization Manager'][
            'Displays']
    }
    assert topics == {
        '/fastdem/mapping/cloud_global',
        '/rubi/heightmap_step_planner/debug/nodes',
        '/rubi/heightmap_step_planner/debug/edges',
        '/rubi/heightmap_step_planner/debug/rejected',
        '/rubi/heightmap_step_planner/debug/revalidation_failure',
        '/rubi/heightmap_step_planner/debug/query_snap',
        '/rubi/heightmap_step_planner/path',
    }
    tools = document['Visualization Manager']['Tools']
    tool_topics = {
        item['Class']: item.get('Topic', {}).get('Value') for item in tools
    }
    assert tool_topics['rviz_default_plugins/SetInitialPose'] == '/initialpose'
    assert tool_topics['rviz_default_plugins/SetGoal'] == '/goal_pose'
    text = rviz.read_text(encoding='utf-8')
    assert 'Navigation 2' not in text
    assert 'nav2_rviz_plugins' not in text


def test_phase1_profiles_only_vary_risk_preferences():
    aggressive = _parameters(CONFIG.parent / 'step_wavefront_phase1_aggressive.yaml')
    safe = _parameters(CONFIG.parent / 'step_wavefront_phase1_safe.yaml')
    frozen = {
        'map_resolution_m', 'max_crossable_height_jump_m',
        'node_sampling_distance_m', 'samples_per_expansion', 'merge_radius_m',
        'neighbor_connection_radius_m', 'goal_connection_distance_m',
    }
    for name in frozen:
        assert aggressive[name] == safe[name]
    assert aggressive['height_cost_weight'] < safe['height_cost_weight']
    assert aggressive['preferred_clearance_radius_m'] < safe[
        'preferred_clearance_radius_m']
    assert aggressive['clearance_cost_weight'] < safe['clearance_cost_weight']
