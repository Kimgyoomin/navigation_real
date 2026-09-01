#!/usr/bin/env python3

import math
import time
import unittest

from geometry_msgs.msg import PoseStamped, Twist
import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
from nav2_msgs.msg import Costmap
from nav_msgs.msg import Path
import pytest
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header


PACKAGE = 'rubi_heightmap_step_wavefront_planner'
COSTMAP = '/compare_test/costmap'
HEIGHTMAP = '/compare_test/heightmap'
GOAL = '/compare_test/goal'
GRID_COSTMAP = '/compare_grid/costmap'
GRID_HEIGHTMAP = '/compare_grid/heightmap'
GRID_GOAL = '/compare_grid/goal'
SAMPLING_COSTMAP = '/compare_sampling/costmap'
SAMPLING_HEIGHTMAP = '/compare_sampling/heightmap'
SAMPLING_GOAL = '/compare_sampling/goal'


@pytest.mark.launch_test
def generate_test_description():
    transform = launch_ros.actions.Node(
        package='tf2_ros', executable='static_transform_publisher',
        arguments=['--x', '0.125', '--y', '0.375', '--z', '0.0',
                   '--frame-id', 'map', '--child-frame-id', 'base_link'])
    comparison = launch_ros.actions.Node(
        package=PACKAGE, executable='hybrid_planner_comparison_node',
        name='hybrid_comparison_launch_test', output='screen', parameters=[{
            'input_costmap_topic': COSTMAP,
            'input_heightmap_topic': HEIGHTMAP,
            'comparison_goal_topic': GOAL,
            'max_costmap_age_s': 10.0, 'max_heightmap_age_s': 10.0,
            'node_evidence_radius_m': 0.06,
            'node_min_observed_cells': 1,
            'node_max_nearest_evidence_distance_m': 0.04,
            'node_max_height_outlier_ratio': 1.0,
            'edge_height_query_radius_m': 0.04,
            'edge_max_height_evidence_gap_m': 0.08,
            'node_sampling_distance_m': 0.15,
            'merge_radius_m': 0.08,
            'neighbor_connection_radius_m': 0.22,
            'goal_connection_distance_m': 0.22,
            'max_nodes': 500, 'max_expansions': 500,
            'post_goal_expansions': 3,
            'sampling.policy': 'original_trg_random_ring',
            'sampling.trg_expand_distance_m': 0.15,
            'sampling.trg_robot_size_m': 0.08,
            'sampling.trg_sample_num': 20,
            'sampling.trg_max_trial_samples': 1000,
            'sampling.trg_random_seed': 42,
            'sampling.trg_neighbor_connection_radius_m': 0.22,
            'sampling.goal_connection_distance_m': 0.22,
            'sampling.max_nodes': 500,
            'sampling.max_expansions': 500,
            'sampling.post_goal_expansions': 3,
        }])
    common = {
        'max_costmap_age_s': 10.0, 'max_heightmap_age_s': 10.0,
        'node_evidence_radius_m': 0.06, 'node_min_observed_cells': 1,
        'node_max_nearest_evidence_distance_m': 0.04,
        'node_max_height_outlier_ratio': 1.0,
        'edge_height_query_radius_m': 0.04,
        'edge_max_height_evidence_gap_m': 0.08,
        'node_sampling_distance_m': 0.15, 'merge_radius_m': 0.08,
        'neighbor_connection_radius_m': 0.22,
        'goal_connection_distance_m': 0.22, 'max_nodes': 500,
        'max_expansions': 500, 'post_goal_expansions': 3,
        'sampling.policy': 'original_trg_random_ring',
        'sampling.trg_expand_distance_m': 0.15,
        'sampling.trg_robot_size_m': 0.08,
        'sampling.trg_sample_num': 20,
        'sampling.trg_max_trial_samples': 1000,
        'sampling.trg_height_threshold_m': 0.08,
        'sampling.trg_collision_threshold': 0.10,
        'sampling.trg_random_seed': 42,
        'sampling.trg_randomize_seed': False,
        'sampling.trg_neighbor_connection_radius_m': 0.22,
        'sampling.goal_connection_distance_m': 0.22,
        'sampling.max_nodes': 500,
        'sampling.max_expansions': 500,
        'sampling.post_goal_expansions': 3,
    }
    grid_only = launch_ros.actions.Node(
        package=PACKAGE, executable='hybrid_planner_comparison_node',
        name='hybrid_grid_only_launch_test', output='screen', parameters=[{
            **common, 'planner_run_mode': 'grid_only',
            'input_costmap_topic': GRID_COSTMAP,
            'input_heightmap_topic': GRID_HEIGHTMAP,
            'comparison_goal_topic': GRID_GOAL,
        }])
    sampling_only = launch_ros.actions.Node(
        package=PACKAGE, executable='hybrid_planner_comparison_node',
        name='hybrid_sampling_only_launch_test', output='screen', parameters=[{
            **common, 'planner_run_mode': 'sampling_only',
            'input_costmap_topic': SAMPLING_COSTMAP,
            'input_heightmap_topic': SAMPLING_HEIGHTMAP,
            'comparison_goal_topic': SAMPLING_GOAL,
        }])
    return launch.LaunchDescription([
        transform, comparison, grid_only, sampling_only,
        launch_testing.actions.ReadyToTest()])


class TestHybridComparison(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('hybrid_comparison_test_client')
        volatile = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        latched = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        cls.costmap_pub = cls.node.create_publisher(Costmap, COSTMAP, latched)
        cls.heightmap_pub = cls.node.create_publisher(PointCloud2, HEIGHTMAP, volatile)
        cls.goal_pub = cls.node.create_publisher(PoseStamped, GOAL, volatile)
        cls.grid_costmap_pub = cls.node.create_publisher(Costmap, GRID_COSTMAP, latched)
        cls.grid_heightmap_pub = cls.node.create_publisher(PointCloud2, GRID_HEIGHTMAP, volatile)
        cls.grid_goal_pub = cls.node.create_publisher(PoseStamped, GRID_GOAL, volatile)
        cls.sampling_costmap_pub = cls.node.create_publisher(
            Costmap, SAMPLING_COSTMAP, latched)
        cls.sampling_heightmap_pub = cls.node.create_publisher(
            PointCloud2, SAMPLING_HEIGHTMAP, volatile)
        cls.sampling_goal_pub = cls.node.create_publisher(
            PoseStamped, SAMPLING_GOAL, volatile)
        cls.grid_paths = []
        cls.sampling_paths = []
        cls.grid_tracking_paths = []
        cls.sampling_tracking_paths = []
        cls.subscriptions = [
            cls.node.create_subscription(
                Path, '/rubi/planner_comparison/grid/path',
                cls.grid_paths.append, latched),
            cls.node.create_subscription(
                Path, '/rubi/planner_comparison/sampling/path',
                cls.sampling_paths.append, latched),
            cls.node.create_subscription(
                Path, '/rubi/planner_comparison/grid/tracking_path',
                cls.grid_tracking_paths.append, latched),
            cls.node.create_subscription(
                Path, '/rubi/planner_comparison/sampling/tracking_path',
                cls.sampling_tracking_paths.append, latched),
        ]

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def _spin_until(self, predicate, timeout=15.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if predicate():
                return
            rclpy.spin_once(self.node, timeout_sec=0.05)
        self.assertTrue(predicate())

    def _publish_inputs(
            self, costmap_pub, heightmap_pub, blocked=False,
            height_step=False):
        now = self.node.get_clock().now().to_msg()
        costmap = Costmap()
        costmap.header.frame_id = 'map'
        costmap.header.stamp = now
        costmap.metadata.size_x = 25
        costmap.metadata.size_y = 15
        costmap.metadata.resolution = 0.05
        costmap.metadata.origin.orientation.w = 1.0
        costmap.data = [0] * (25 * 15)
        if blocked:
            costmap.data[7 * 25 + 11] = 254
        points = [(0.025 + 0.05 * x, 0.025 + 0.05 * y,
                   0.10 if height_step and x >= 12 else 0.0)
                  for y in range(15) for x in range(25)]
        cloud = point_cloud2.create_cloud_xyz32(
            Header(stamp=now, frame_id='map'), points)
        costmap_pub.publish(costmap)
        heightmap_pub.publish(cloud)
        for _ in range(10):
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def _goal(self):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.node.get_clock().now().to_msg()
        goal.pose.position.x = 1.075
        goal.pose.position.y = 0.375
        goal.pose.orientation.z = math.sin(0.5)
        goal.pose.orientation.w = math.cos(0.5)
        return goal

    def test_1_grid_only_does_not_publish_sampling(self, proc_output):
        self._spin_until(lambda: self.grid_goal_pub.get_subscription_count() > 0)
        self._publish_inputs(self.grid_costmap_pub, self.grid_heightmap_pub)
        grid_before, sampling_before = len(self.grid_paths), len(self.sampling_paths)
        self.grid_goal_pub.publish(self._goal())
        self._spin_until(lambda: len(self.grid_paths) > grid_before)
        self._spin_until(lambda: self.grid_tracking_paths and
                         self.grid_tracking_paths[-1].poses)
        for _ in range(10):
            rclpy.spin_once(self.node, timeout_sec=0.02)
        self.assertEqual(len(self.sampling_paths), sampling_before)
        proc_output.assertWaitFor(expected_output='planner=sampling status=not_run', timeout=5.0)

    def test_2_sampling_only_does_not_publish_grid(self, proc_output):
        self._spin_until(lambda: self.sampling_goal_pub.get_subscription_count() > 0)
        self._publish_inputs(self.sampling_costmap_pub, self.sampling_heightmap_pub)
        grid_before, sampling_before = len(self.grid_paths), len(self.sampling_paths)
        self.sampling_goal_pub.publish(self._goal())
        self._spin_until(lambda: len(self.sampling_paths) > sampling_before)
        self._spin_until(lambda: self.sampling_tracking_paths and
                         self.sampling_tracking_paths[-1].poses)
        for _ in range(10):
            rclpy.spin_once(self.node, timeout_sec=0.02)
        self.assertEqual(len(self.grid_paths), grid_before)
        proc_output.assertWaitFor(expected_output='planner=grid status=not_run', timeout=5.0)

    def test_3_same_snapshot_pair_publishes_both_paths(self, proc_output):
        self._spin_until(lambda: (
            self.costmap_pub.get_subscription_count() > 0
            and self.heightmap_pub.get_subscription_count() > 0
            and self.goal_pub.get_subscription_count() > 0))
        self._publish_inputs(self.costmap_pub, self.heightmap_pub)
        self.goal_pub.publish(self._goal())
        self._spin_until(lambda: (
            self.grid_paths and self.grid_paths[-1].poses
            and self.sampling_paths and self.sampling_paths[-1].poses))
        self._spin_until(lambda: (
            self.grid_tracking_paths and self.grid_tracking_paths[-1].poses
            and self.sampling_tracking_paths and
            self.sampling_tracking_paths[-1].poses))
        for path in (self.grid_paths[-1], self.sampling_paths[-1]):
            self.assertEqual(path.header.frame_id, 'map')
            final = path.poses[-1].pose.orientation
            self.assertAlmostEqual(
                abs(final.z * math.sin(0.5) + final.w * math.cos(0.5)),
                1.0, places=6)
        proc_output.assertWaitFor(
            expected_output='comparison_snapshot costmap_generation=1', timeout=5.0)
        proc_output.assertWaitFor(expected_output='planner=grid success=true', timeout=5.0)
        proc_output.assertWaitFor(expected_output='planner=sampling success=true', timeout=5.0)
        self.assertEqual(
            len(self.node.get_publishers_info_by_topic('/cmd_vel')), 0)

    def test_4_content_dedup_and_hard_costmap_update_auto_replan(self, proc_output):
        settle = time.monotonic() + 0.5
        while time.monotonic() < settle:
            rclpy.spin_once(self.node, timeout_sec=0.05)
        before = len(self.grid_tracking_paths)
        self._publish_inputs(self.grid_costmap_pub, self.grid_heightmap_pub)
        deadline = time.monotonic() + 1.8
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
        self.assertEqual(len(self.grid_tracking_paths), before)

        self._publish_inputs(
            self.grid_costmap_pub, self.grid_heightmap_pub, blocked=True)
        self._spin_until(lambda: any(
            not path.poses for path in self.grid_tracking_paths[before:]), timeout=10.0)
        self._spin_until(lambda: (
            self.grid_tracking_paths and self.grid_tracking_paths[-1].poses), timeout=10.0)
        proc_output.assertWaitFor(
            expected_output='action=stop_then_replan', timeout=5.0)

    def test_5_over_limit_height_update_stops_until_map_recovers(self, proc_output):
        self._publish_inputs(self.grid_costmap_pub, self.grid_heightmap_pub)
        before = len(self.grid_tracking_paths)
        self.grid_goal_pub.publish(self._goal())
        self._spin_until(lambda: (
            len(self.grid_tracking_paths) > before and
            self.grid_tracking_paths[-1].poses), timeout=10.0)

        update_start = len(self.grid_tracking_paths)
        self._publish_inputs(
            self.grid_costmap_pub, self.grid_heightmap_pub,
            height_step=True)
        self._spin_until(lambda: any(
            not path.poses for path in self.grid_tracking_paths[update_start:]),
            timeout=10.0)
        stop_index = next(
            index for index, path in enumerate(
                self.grid_tracking_paths[update_start:], start=update_start)
            if not path.poses)
        settle = time.monotonic() + 1.2
        while time.monotonic() < settle:
            rclpy.spin_once(self.node, timeout_sec=0.05)
        self.assertFalse(any(
            path.poses for path in self.grid_tracking_paths[stop_index + 1:]))
        proc_output.assertWaitFor(
            expected_output='invalid_reason=step_limit', timeout=5.0)

        self._publish_inputs(self.grid_costmap_pub, self.grid_heightmap_pub)
        self._spin_until(lambda: (
            len(self.grid_tracking_paths) > stop_index + 1 and
            self.grid_tracking_paths[-1].poses), timeout=10.0)


@launch_testing.post_shutdown_test()
class TestProcesses(unittest.TestCase):
    def test_exit(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
