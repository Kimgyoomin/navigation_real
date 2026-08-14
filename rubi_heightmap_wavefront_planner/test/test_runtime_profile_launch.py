#!/usr/bin/env python3

# Copyright 2026
# Licensed under the Apache License, Version 2.0

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


PACKAGE_NAME = 'rubi_heightmap_wavefront_planner'
COMPILED_DEFAULT_NODE = 'wavefront_runtime_compiled_defaults'
INSTALLED_PROFILE_NODE = 'rubi_heightmap_wavefront_planner'
PARAMETER_NAMES = [
    'planner_mode',
    'base_frame',
    'map_resolution_m',
    'lattice_tolerance_m',
    'max_grid_cells',
    'transform_timeout_s',
    'edge_check_spacing_m',
    'node_sampling_distance_m',
    'samples_per_expansion',
    'merge_radius_m',
    'neighbor_connection_radius_m',
    'goal_connection_distance_m',
    'max_build_time_ms',
    'path_output_spacing_m',
]
EXPECTED_PROFILE = {
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
    'max_build_time_ms': 5000,
    'path_output_spacing_m': 0.05,
}


class AsyncParameterClient:
    """Humble-compatible asynchronous get-parameters client."""

    def __init__(self, node, remote_node_name):
        service_name = f'/{remote_node_name.strip("/")}/get_parameters'
        self._client = node.create_client(GetParameters, service_name)

    def wait_for_service(self, timeout_sec):
        return self._client.wait_for_service(timeout_sec=timeout_sec)

    def get_parameters(self, names):
        request = GetParameters.Request()
        request.names = names
        return self._client.call_async(request)


def parameter_value(message):
    parameter_type = Parameter.Type(value=message.type)
    if parameter_type == Parameter.Type.STRING:
        return message.string_value
    if parameter_type == Parameter.Type.INTEGER:
        return message.integer_value
    if parameter_type == Parameter.Type.DOUBLE:
        return message.double_value
    raise AssertionError(f'Unexpected parameter type: {parameter_type}')


@pytest.mark.launch_test
def generate_test_description():
    package_share = Path(get_package_share_directory(PACKAGE_NAME))
    installed_launch = package_share / 'launch' / 'wavefront_v0.launch.py'

    compiled_defaults = launch_ros.actions.Node(
        package=PACKAGE_NAME,
        executable='wavefront_planner_node',
        name=COMPILED_DEFAULT_NODE,
        output='screen',
    )
    installed_profile = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            str(installed_launch)
        ),
        launch_arguments={'launch_rviz': 'false'}.items(),
    )

    return launch.LaunchDescription(
        [
            compiled_defaults,
            installed_profile,
            launch_testing.actions.ReadyToTest(),
        ]
    )


class TestRuntimeProfile(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('wavefront_runtime_profile_test_client')

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def _read_parameters(self, remote_node_name):
        client = AsyncParameterClient(self.node, remote_node_name)
        self.assertTrue(
            client.wait_for_service(timeout_sec=10.0),
            f'Parameter service for {remote_node_name} was not available',
        )
        future = client.get_parameters(PARAMETER_NAMES)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=10.0)
        self.assertTrue(
            future.done(),
            f'Parameter request for {remote_node_name} timed out',
        )
        self.assertIsNone(future.exception())
        parameter_values = future.result().values
        self.assertEqual(len(PARAMETER_NAMES), len(parameter_values))
        return {
            name: parameter_value(value)
            for name, value in zip(PARAMETER_NAMES, parameter_values)
        }

    def _assert_profile(self, actual):
        self.assertEqual(set(EXPECTED_PROFILE), set(actual))
        for name, expected in EXPECTED_PROFILE.items():
            if isinstance(expected, float):
                self.assertTrue(
                    math.isclose(
                        actual[name], expected, rel_tol=0.0, abs_tol=1.0e-12
                    ),
                    f'{name}: expected {expected}, got {actual[name]}',
                )
            else:
                self.assertEqual(expected, actual[name], name)

    def test_compiled_defaults_and_installed_profile(self):
        self._assert_profile(self._read_parameters(COMPILED_DEFAULT_NODE))
        self._assert_profile(self._read_parameters(INSTALLED_PROFILE_NODE))


@launch_testing.post_shutdown_test()
class TestRuntimeProfileProcessesExitCleanly(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
