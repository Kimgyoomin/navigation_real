from pathlib import Path

import yaml


CONFIG = Path(__file__).resolve().parents[1] / 'config' / 'step_wavefront_v0.yaml'


def _parameters(path):
    with path.open(encoding='utf-8') as stream:
        return yaml.safe_load(stream)[
            'rubi_heightmap_step_wavefront_planner']['ros__parameters']


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
        'max_rejected_markers': 5000,
    }
    for name, value in expected.items():
        assert parameters[name] == value


def test_rviz_has_only_top_level_supported_displays():
    rviz = CONFIG.parents[1] / 'rviz' / 'step_wavefront_v0.rviz'
    with rviz.open(encoding='utf-8') as stream:
        document = yaml.safe_load(stream)
    classes = [item['Class'] for item in document['Visualization Manager']['Displays']]
    assert 'rviz_default_plugins/Group' not in classes
    assert 'rviz_default_plugins/PointCloud2' in classes
    assert 'rviz_default_plugins/Path' in classes
    assert classes.count('rviz_default_plugins/MarkerArray') == 3
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
