from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT / 'config' / 'hybrid_global_costmap.yaml'


def _parameters():
    document = yaml.safe_load(CONFIG.read_text(encoding='utf-8'))
    return document['global_costmap']['global_costmap']['ros__parameters']


def test_static_inflation_only_contract():
    parameters = _parameters()
    assert parameters['plugins'] == ['static_layer', 'inflation_layer']
    assert 'obstacle_layer' not in parameters
    assert parameters['static_layer']['map_topic'] == '/map'
    assert parameters['static_layer']['plugin'] == 'nav2_costmap_2d::StaticLayer'
    assert parameters['inflation_layer']['plugin'] == 'nav2_costmap_2d::InflationLayer'
    assert parameters['inflation_layer']['inflation_radius'] == 0.70
    assert parameters['inflation_layer']['cost_scaling_factor'] == 1.0


def test_global_geometry_and_publish_contract():
    parameters = _parameters()
    assert parameters['resolution'] == 0.05
    assert parameters['global_frame'] == 'map'
    assert parameters['robot_base_frame'] == 'base_link'
    assert parameters['track_unknown_space'] is True
    assert parameters['rolling_window'] is False
    assert parameters['always_send_full_costmap'] is True


def test_original_trg_demo_merge_radius_is_below_sampling_radius():
    config = ROOT / 'config' / 'hybrid_grid_trg_comparison_v1.yaml'
    parameters = yaml.safe_load(config.read_text(encoding='utf-8'))[
        'rubi_hybrid_planner_comparison']['ros__parameters']
    assert parameters['sampling.policy'] == 'original_trg_random_ring'
    assert (parameters['sampling.trg_robot_size_m'] <
            parameters['sampling.trg_expand_distance_m'])


def test_navigation_launches_include_self_contained_bringup():
    launch_dir = ROOT / 'launch'
    for filename, mode in (
            ('hybrid_grid_navigation.launch.py', 'grid_only'),
            ('hybrid_sampling_navigation.launch.py', 'sampling_only'),
            ('hybrid_grid_trg_comparison.launch.py', 'both')):
        text = (launch_dir / filename).read_text(encoding='utf-8')
        assert 'hybrid_costmap_bringup.launch.py' in text
        assert "DeclareLaunchArgument('launch_map_costmap', default_value='true')" in text
        assert "'map_yaml': map_yaml" in text
        assert "'costmap_params': costmap_params" in text
        assert f"'planner_run_mode': '{mode}'" in text
    comparison = (launch_dir / 'hybrid_grid_trg_comparison.launch.py').read_text(
        encoding='utf-8')
    assert 'simple_pure_pursuit_controller' not in comparison


def test_navigation_controller_can_only_be_disabled_explicitly_for_tests():
    launch_dir = ROOT / 'launch'
    for filename in (
            'hybrid_grid_navigation.launch.py',
            'hybrid_sampling_navigation.launch.py'):
        text = (launch_dir / filename).read_text(encoding='utf-8')
        assert "DeclareLaunchArgument('launch_controller', default_value='true')" in text
        assert 'condition=IfCondition(launch_controller)' in text
        assert text.count("executable='simple_pure_pursuit_controller'") == 1
