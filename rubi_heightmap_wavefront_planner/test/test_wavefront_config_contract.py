import math
from pathlib import Path

import yaml


CONFIG_PATH = (
    Path(__file__).resolve().parents[1] / 'config' / 'wavefront_v0.yaml'
)
RVIZ_PATH = (
    Path(__file__).resolve().parents[1] / 'rviz' / 'wavefront_v0.rviz'
)


def test_wavefront_v0_fastdem_profile_contract():
    with CONFIG_PATH.open(encoding='utf-8') as config_file:
        document = yaml.safe_load(config_file)

    parameters = document['rubi_heightmap_wavefront_planner']['ros__parameters']
    expected = {
        'map_resolution_m': 0.05,
        'lattice_tolerance_m': 0.01,
        'edge_check_spacing_m': 0.025,
        'node_sampling_distance_m': 0.30,
        'samples_per_expansion': 20,
        'merge_radius_m': 0.20,
        'neighbor_connection_radius_m': 0.45,
        'goal_connection_distance_m': 0.45,
        'max_build_time_ms': 5000,
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
