import math
from pathlib import Path

import yaml


CONFIG_DIRECTORY = Path(__file__).resolve().parents[1] / 'config'
WAVEFRONT_CONFIG_PATH = CONFIG_DIRECTORY / 'wavefront_v0.yaml'
RRT_STAR_CONFIG_PATH = CONFIG_DIRECTORY / 'rrt_star_v0.yaml'
RVIZ_PATH = (
    Path(__file__).resolve().parents[1] / 'rviz' / 'wavefront_v0.rviz'
)


def _parameters(config_path):
    with config_path.open(encoding='utf-8') as config_file:
        document = yaml.safe_load(config_file)
    return document['rubi_heightmap_wavefront_planner']['ros__parameters']


def test_profiles_match_except_for_planner_mode():
    wavefront = _parameters(WAVEFRONT_CONFIG_PATH)
    rrt_star = _parameters(RRT_STAR_CONFIG_PATH)

    assert wavefront['planner_mode'] == 'wavefront'
    assert rrt_star['planner_mode'] == 'rrt_star'
    assert {
        name: value for name, value in wavefront.items() if name != 'planner_mode'
    } == {
        name: value for name, value in rrt_star.items() if name != 'planner_mode'
    }


def test_wavefront_v0_runtime_profile_contract():
    parameters = _parameters(WAVEFRONT_CONFIG_PATH)
    expected = {
        'planner_mode': 'wavefront',
        'base_frame': 'base_link',
        'map_resolution_m': 0.05,
        'lattice_tolerance_m': 0.01,
        'max_grid_cells': 5000000,
        'transform_timeout_s': 0.20,
        'edge_check_spacing_m': 0.025,
        'node_sampling_distance_m': 0.30,
        'samples_per_expansion': 20,
        'merge_radius_m': 0.20,
        'neighbor_connection_radius_m': 0.45,
        'goal_connection_distance_m': 0.45,
        'max_nodes': 4000,
        'max_expansions': 4000,
        'max_build_time_ms': 5000,
        'path_output_spacing_m': 0.05,
    }

    for name, value in expected.items():
        assert parameters[name] == value


def test_rrt_star_v0_five_centimeter_profile_contract():
    parameters = _parameters(RRT_STAR_CONFIG_PATH)
    expected = {
        'map_resolution_m': 0.05,
        'lattice_tolerance_m': 0.01,
        'edge_check_spacing_m': 0.025,
        'support_radius_m': 0.20,
        'minimum_observed_support_ratio': 1.00,
        'max_step_height_m': 0.08,
        'max_slope_deg': 15.0,
        'rrt_star.max_iterations': 5000,
        'rrt_star.goal_bias': 0.05,
        'rrt_star.steer_distance_m': 0.50,
        'rrt_star.rewire_radius_min_m': 0.30,
        'rrt_star.rewire_radius_max_m': 1.00,
        'rrt_star.goal_connection_distance_m': 0.75,
        'rrt_star.max_nodes': 4000,
        'rrt_star.max_planning_time_ms': 2000,
        'rrt_star.stop_on_first_solution': False,
        'rrt_star.random_seed': 42,
    }

    for name, value in expected.items():
        assert parameters[name] == value

    assert math.isclose(
        parameters['edge_check_spacing_m'],
        parameters['map_resolution_m'] / 2.0,
        rel_tol=0.0,
        abs_tol=1.0e-12,
    )


def test_wavefront_v0_rviz_contract():
    with RVIZ_PATH.open(encoding='utf-8') as rviz_file:
        document = yaml.safe_load(rviz_file)

    displays = document['Visualization Manager']['Displays']
    display_by_name = {display['Name']: display for display in displays}
    assert 'Valid Edges' in display_by_name
    assert 'Rejected Nodes / Edges' in display_by_name

    final_path = display_by_name['Final Path']
    assert final_path['Color'] == '255; 255; 0'
    assert final_path['Pose Color'] == '255; 255; 0'
