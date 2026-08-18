#!/usr/bin/env python3

import math
from pathlib import Path
import unittest

from ament_index_python.packages import get_package_share_directory
import launch
import launch.actions
import launch.launch_description_sources
import launch_ros.actions
import launch_testing
import launch_testing.actions
import pytest
from rcl_interfaces.srv import GetParameters
import rclpy
from rclpy.parameter import Parameter


PACKAGE = 'rubi_heightmap_step_wavefront_planner'
NAMES = [
    'base_frame', 'map_resolution_m', 'lattice_tolerance_m', 'max_grid_cells',
    'transform_timeout_s', 'hard_clearance_radius_m', 'edge_check_spacing_m',
    'max_crossable_height_jump_m', 'height_noise_floor_m',
    'height_cost_exponent', 'distance_weight', 'height_cost_weight',
    'node_sampling_distance_m', 'samples_per_expansion', 'merge_radius_m',
    'neighbor_connection_radius_m', 'goal_connection_distance_m', 'max_nodes',
    'max_expansions', 'max_graph_build_time_ms', 'post_goal_expansions',
    'path_output_spacing_m', 'path_invalid_confirmations',
    'max_rejected_markers',
]
EXPECTED = {
    'base_frame': 'base_link', 'map_resolution_m': 0.05,
    'lattice_tolerance_m': 0.01, 'max_grid_cells': 5000000,
    'transform_timeout_s': 0.20, 'hard_clearance_radius_m': 0.20,
    'edge_check_spacing_m': 0.025, 'max_crossable_height_jump_m': 0.08,
    'height_noise_floor_m': 0.01, 'height_cost_exponent': 2.0,
    'distance_weight': 1.0, 'height_cost_weight': 5.0,
    'node_sampling_distance_m': 0.30, 'samples_per_expansion': 20,
    'merge_radius_m': 0.20, 'neighbor_connection_radius_m': 0.45,
    'goal_connection_distance_m': 0.45, 'max_nodes': 4000,
    'max_expansions': 4000, 'max_graph_build_time_ms': 5000,
    'post_goal_expansions': 50, 'path_output_spacing_m': 0.05,
    'path_invalid_confirmations': 2, 'max_rejected_markers': 5000,
}


def value(message):
    kind = Parameter.Type(value=message.type)
    if kind == Parameter.Type.STRING:
        return message.string_value
    if kind == Parameter.Type.INTEGER:
        return message.integer_value
    if kind == Parameter.Type.DOUBLE:
        return message.double_value
    raise AssertionError(f'unexpected parameter type {kind}')


@pytest.mark.launch_test
def generate_test_description():
    share = Path(get_package_share_directory(PACKAGE))
    direct = launch_ros.actions.Node(
        package=PACKAGE, executable='step_wavefront_planner_node',
        name='step_wavefront_compiled_defaults', output='screen')
    installed = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            str(share / 'launch' / 'step_wavefront_v0.launch.py')),
        launch_arguments={'launch_rviz': 'false'}.items())
    return launch.LaunchDescription([
        direct, installed, launch_testing.actions.ReadyToTest()])


class TestRuntimeProfile(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('step_wavefront_runtime_profile_client')

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def _read(self, remote):
        service = self.node.create_client(
            GetParameters, f'/{remote}/get_parameters')
        self.assertTrue(service.wait_for_service(timeout_sec=10.0))
        request = GetParameters.Request()
        request.names = NAMES
        future = service.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=10.0)
        self.assertTrue(future.done())
        return dict(zip(NAMES, [value(item) for item in future.result().values]))

    def test_compiled_and_installed_profiles_match(self):
        for remote in (
            'step_wavefront_compiled_defaults',
            'rubi_heightmap_step_wavefront_planner',
        ):
            actual = self._read(remote)
            for name, expected in EXPECTED.items():
                if isinstance(expected, float):
                    self.assertTrue(math.isclose(
                        actual[name], expected, rel_tol=0.0, abs_tol=1e-12), name)
                else:
                    self.assertEqual(actual[name], expected, name)


@launch_testing.post_shutdown_test()
class TestProcesses(unittest.TestCase):

    def test_exit(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
