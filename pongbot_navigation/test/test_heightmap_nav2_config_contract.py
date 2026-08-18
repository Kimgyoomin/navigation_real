from pathlib import Path
import xml.etree.ElementTree as ET

import yaml


PACKAGE = Path(__file__).parents[1]


def test_plugin_overlay_and_bt_identifiers_match():
    plugin = ET.parse(PACKAGE / 'plugins.xml').getroot().find('class')
    overlay = yaml.safe_load(
        (PACKAGE / 'config/nav2_rubi_heightmap_wavefront_overlay.yaml').read_text())
    params = overlay['planner_server']['ros__parameters']
    tree = ET.parse(
        PACKAGE / 'behavior_trees/navigate_to_pose_rubi_heightmap.xml').getroot()
    compute = tree.find('.//ComputePathToPose')
    rate = tree.find('.//RateController')
    assert plugin.attrib['name'] == params['HeightmapWavefront']['plugin']
    assert params['planner_plugins'] == ['HeightmapWavefront']
    assert compute.attrib['planner_id'] == 'HeightmapWavefront'
    assert float(rate.attrib['hz']) == 1.0
    assert params['expected_planner_frequency'] == 1.0


def test_overlay_is_narrow_and_shared_values_match_standalone():
    overlay = yaml.safe_load(
        (PACKAGE / 'config/nav2_rubi_heightmap_wavefront_overlay.yaml').read_text())
    assert set(overlay) == {'planner_server', 'bt_navigator'}
    text = (PACKAGE / 'config/nav2_rubi_heightmap_wavefront_overlay.yaml').read_text()
    for forbidden in ('controller_server:', 'local_costmap:', 'global_costmap:'):
        assert forbidden not in text
    standalone = yaml.safe_load(
        (PACKAGE.parent / 'rubi_heightmap_wavefront_planner/config/wavefront_v0.yaml').read_text())
    shared = standalone['rubi_heightmap_wavefront_planner']['ros__parameters']
    plugin = overlay['planner_server']['ros__parameters']['HeightmapWavefront']
    keys = {
        'map_resolution_m', 'lattice_tolerance_m', 'reject_duplicate_cells',
        'max_grid_cells', 'pca_analysis_radius_m', 'pca_min_points',
        'minimum_observed_support_ratio', 'max_slope_deg', 'max_roughness_m',
        'max_step_height_m', 'edge_check_spacing_m',
        'check_footprint_along_edge', 'node_sampling_distance_m',
        'samples_per_expansion', 'merge_radius_m',
        'neighbor_connection_radius_m', 'goal_connection_distance_m',
        'max_nodes', 'max_expansions', 'stop_when_goal_connected',
        'distance_weight', 'slope_risk_weight', 'step_risk_weight',
        'roughness_risk_weight', 'path_output_spacing_m',
    }
    for key in keys:
        assert plugin[key] == shared[key]
    assert plugin['support_radius_m'] == 0.26
    assert shared['support_radius_m'] == 0.20
    assert plugin['max_build_time_ms'] == 800
    assert shared['max_build_time_ms'] == 5000


def test_new_launch_keeps_base_only_for_controller_and_costmap_consumers():
    launch = (PACKAGE / 'launch/rubi_navigation_heightmap_dwb.launch.py').read_text()
    assert 'parameters=[configured_base_params, configured_heightmap_overlay]' in launch
    assert launch.count('parameters=[configured_base_params]') == 3
    for argument in (
        'map_yaml', 'base_params_file', 'heightmap_overlay_file',
        'use_sim_time', 'autostart',
    ):
        assert f'"{argument}"' in launch


def test_existing_dwb_profile_remains_selected_and_untouched_by_overlay():
    base = yaml.safe_load((PACKAGE / 'config/nav2_rubi_dwb.yaml').read_text())
    controller = base['controller_server']['ros__parameters']
    follow = controller['FollowPath']
    assert controller['controller_plugins'] == ['FollowPath']
    assert follow['plugin'] == (
        'nav2_rotation_shim_controller::RotationShimController')
    assert follow['primary_controller'] == 'dwb_core::DWBLocalPlanner'
