#!/usr/bin/env python3

import os
import time
import unittest

from ament_index_python.packages import get_package_share_directory
import launch
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_ros.actions
import launch_testing
import launch_testing.actions
from lifecycle_msgs.srv import GetState
from nav2_msgs.srv import ManageLifecycleNodes
from nav2_msgs.msg import Costmap
from nav_msgs.msg import OccupancyGrid
import pytest
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


PACKAGE = 'rubi_heightmap_step_wavefront_planner'


@pytest.mark.launch_test
def generate_test_description():
    share = get_package_share_directory(PACKAGE)
    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            share, 'launch', 'hybrid_costmap_bringup.launch.py')),
        launch_arguments={'use_sim_time': 'false', 'autostart': 'true'}.items())
    transform = launch_ros.actions.Node(
        package='tf2_ros', executable='static_transform_publisher',
        arguments=['--x', '0.0', '--y', '0.0', '--z', '0.0',
                   '--frame-id', 'map', '--child-frame-id', 'base_link'])
    return launch.LaunchDescription([
        transform, bringup, launch_testing.actions.ReadyToTest()])


class TestHybridCostmapBringup(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('hybrid_costmap_bringup_test_client')
        latched = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        cls.maps = []
        cls.costmaps = []
        cls.raw_costmaps = []
        cls.subscriptions = [
            cls.node.create_subscription(
                OccupancyGrid, '/map', cls.maps.append, latched),
            cls.node.create_subscription(
                OccupancyGrid, '/global_costmap/costmap',
                cls.costmaps.append, latched),
            cls.node.create_subscription(
                Costmap, '/global_costmap/costmap_raw',
                cls.raw_costmaps.append, latched),
        ]

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def _spin_until(self, predicate, timeout=60.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if predicate():
                return
            rclpy.spin_once(self.node, timeout_sec=0.1)
        self.assertTrue(predicate())

    def _lifecycle_state(self, node_name):
        client = self.node.create_client(GetState, f'{node_name}/get_state')
        self.assertTrue(client.wait_for_service(timeout_sec=20.0))
        future = client.call_async(GetState.Request())
        self._spin_until(future.done, timeout=20.0)
        return future.result().current_state.label

    def _wait_lifecycle_active(self, node_name):
        state = [None]

        def active():
            state[0] = self._lifecycle_state(node_name)
            return state[0] == 'active'

        self._spin_until(active, timeout=30.0)
        return state[0]

    def test_1_lifecycle_nodes_are_active(self):
        self.assertEqual(self._wait_lifecycle_active('/map_server'), 'active')
        self.assertEqual(
            self._wait_lifecycle_active('/global_costmap/global_costmap'), 'active')

    def test_2_map_and_costmap_topics_have_expected_contract(self):
        self._spin_until(lambda: self.maps and self.costmaps and self.raw_costmaps)
        map_msg = self.maps[-1]
        costmap_msg = self.costmaps[-1]
        raw_msg = self.raw_costmaps[-1]
        self.assertEqual(map_msg.header.frame_id, 'map')
        self.assertAlmostEqual(map_msg.info.resolution, 0.05, places=6)
        self.assertGreater(map_msg.info.width, 0)
        self.assertGreater(map_msg.info.height, 0)
        self.assertEqual(costmap_msg.header.frame_id, 'map')
        self.assertAlmostEqual(costmap_msg.info.resolution, 0.05, places=6)
        self.assertEqual(raw_msg.header.frame_id, 'map')
        self.assertAlmostEqual(raw_msg.metadata.resolution, 0.05, places=6)
        self.assertGreater(raw_msg.metadata.size_x, 0)
        self.assertGreater(raw_msg.metadata.size_y, 0)

    def test_3_raw_costmap_contains_lethal_and_soft_inflation(self):
        self._spin_until(lambda: self.raw_costmaps)
        values = set(self.raw_costmaps[-1].data)
        self.assertIn(254, values)
        self.assertTrue(any(1 <= value <= 252 for value in values))
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        self.assertEqual(
            self._lifecycle_state('/global_costmap/global_costmap'), 'active')
        client = self.node.create_client(
            ManageLifecycleNodes,
            '/lifecycle_manager_hybrid_costmap/manage_nodes')
        self.assertTrue(client.wait_for_service(timeout_sec=10.0))
        request = ManageLifecycleNodes.Request()
        request.command = ManageLifecycleNodes.Request.SHUTDOWN
        future = client.call_async(request)
        self._spin_until(future.done, timeout=20.0)
        self.assertTrue(future.result().success)


@launch_testing.post_shutdown_test()
class TestProcesses(unittest.TestCase):
    def test_exit(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
