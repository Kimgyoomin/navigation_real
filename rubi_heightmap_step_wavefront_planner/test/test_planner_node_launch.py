#!/usr/bin/env python3

import math
import time
import unittest

from geometry_msgs.msg import PoseStamped
import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
from nav_msgs.msg import Path
import pytest
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray


PACKAGE = 'rubi_heightmap_step_wavefront_planner'
MAP = 'map'
BASE = 'base_link'
CLOUD = '/step_test/cloud'
GOAL = '/step_test/goal'
PATH = '/step_test/path'
NODES = '/step_test/nodes'
EDGES = '/step_test/edges'
REJECTED = '/step_test/rejected'
FAILURE = '/step_test/revalidation_failure'
QUERY_SNAP = '/step_test/query_snap'


@pytest.mark.launch_test
def generate_test_description():
    static_tf = launch_ros.actions.Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='step_test_tf', output='screen', arguments=[
            '--x', '-0.60', '--y', '0.0', '--z', '0.0',
            '--frame-id', MAP, '--child-frame-id', BASE])
    planner = launch_ros.actions.Node(
        package=PACKAGE, executable='step_wavefront_planner_node',
        name='step_wavefront_launch_test', output='screen', parameters=[{
            'input_cloud_topic': CLOUD, 'goal_topic': GOAL,
            'path_topic': PATH, 'debug_nodes_topic': NODES,
            'debug_edges_topic': EDGES, 'debug_rejected_topic': REJECTED,
            'debug_revalidation_failure_topic': FAILURE,
            'debug_query_snap_topic': QUERY_SNAP,
            'snap_start_to_valid_map': True,
            'snap_goal_to_valid_map': True,
            'start_snap_radius_m': 0.30, 'goal_snap_radius_m': 0.25,
            'base_frame': BASE, 'hard_clearance_radius_m': 0.10,
            'max_grid_cells': 100000, 'post_goal_expansions': 3,
            'max_graph_build_time_ms': 5000,
        }])
    return (
        launch.LaunchDescription([
            static_tf, planner, launch_testing.actions.ReadyToTest()]),
        {'planner': planner},
    )


class TestPlannerNode(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('step_wavefront_launch_test_client')
        volatile = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        latched = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        cls.cloud_pub = cls.node.create_publisher(PointCloud2, CLOUD, volatile)
        cls.goal_pub = cls.node.create_publisher(PoseStamped, GOAL, volatile)
        cls.paths = []
        cls.nodes = []
        cls.edges = []
        cls.rejected = []
        cls.failures = []
        cls.query_snaps = []
        cls.subscriptions = [
            cls.node.create_subscription(Path, PATH, cls.paths.append, latched),
            cls.node.create_subscription(MarkerArray, NODES, cls.nodes.append, latched),
            cls.node.create_subscription(MarkerArray, EDGES, cls.edges.append, latched),
            cls.node.create_subscription(
                MarkerArray, REJECTED, cls.rejected.append, latched),
            cls.node.create_subscription(
                Marker, FAILURE, cls.failures.append, latched),
            cls.node.create_subscription(
                MarkerArray, QUERY_SNAP, cls.query_snaps.append, latched),
        ]

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def _wait(self, predicate, timeout=15.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if predicate():
                return
            rclpy.spin_once(self.node, timeout_sec=0.05)
        self.assertTrue(predicate(), 'timed out waiting for planner output')

    def _spin_for(self, duration):
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    @staticmethod
    def _cloud(kind):
        points = []
        for y in range(-16, 17):
            for x in range(-24, 25):
                if kind in ('unknown', 'unknown_a', 'unknown_b') and x == 0:
                    continue
                z = 0.0
                if kind == 'crossable' and x >= 0:
                    z = 0.045
                if kind == 'over' and x >= 0:
                    z = 0.081
                if kind == 'outside' and x == 20 and y == 15:
                    z = 0.081
                if kind == 'unknown_b' and x == 20 and y == 15:
                    z = 0.01
                if kind == 'query_holes' and y == 0 and x in (-12, 12):
                    continue
                points.append((0.05 * x, 0.05 * y, z))
        return point_cloud2.create_cloud_xyz32(Header(frame_id=MAP), points)

    def _goal(self, x=0.60, y=0.0, yaw=0.70):
        message = PoseStamped()
        message.header.frame_id = MAP
        message.pose.position.x = x
        message.pose.position.y = y
        message.pose.orientation.z = 3.0 * math.sin(0.5 * yaw)
        message.pose.orientation.w = 3.0 * math.cos(0.5 * yaw)
        return message

    def _assert_full_reset(self, checkpoint, expected_frame):
        self._wait(lambda: (
            len(self.paths) > checkpoint[0]
            and len(self.nodes) > checkpoint[1]
            and len(self.edges) > checkpoint[2]
            and len(self.rejected) > checkpoint[3]
            and len(self.paths[-1].poses) == 0
            and all(
                len(output[-1].markers) == 1
                and output[-1].markers[0].action == Marker.DELETEALL
                for output in (self.nodes, self.edges, self.rejected)
            )
        ))
        path = self.paths[-1]
        self.assertEqual(path.header.frame_id, expected_frame)
        self.assertEqual(len(path.poses), 0)
        stamp = (path.header.stamp.sec, path.header.stamp.nanosec)
        for output in (self.nodes[-1], self.edges[-1], self.rejected[-1]):
            self.assertEqual(len(output.markers), 1)
            marker = output.markers[0]
            self.assertEqual(marker.action, Marker.DELETEALL)
            self.assertEqual(marker.header.frame_id, expected_frame)
            self.assertEqual(
                (marker.header.stamp.sec, marker.header.stamp.nanosec), stamp)

    def _publish_and_wait_path(self, cloud_kind, expect_nonempty):
        old_count = len(self.paths)
        self.cloud_pub.publish(self._cloud(cloud_kind))
        self.goal_pub.publish(self._goal())
        self._wait(lambda: len(self.paths) > old_count)
        if expect_nonempty:
            self._wait(lambda: self.paths and len(self.paths[-1].poses) > 0)
        else:
            self._wait(lambda: self.paths and len(self.paths[-1].poses) == 0)
        return self.paths[-1]

    def test_flat_crossable_and_hard_barriers(
        self, proc_info, proc_output, planner
    ):
        proc_info.assertWaitForStartup(process=planner, timeout=10.0)
        self._wait(lambda: self.cloud_pub.get_subscription_count() > 0)
        self._wait(lambda: self.goal_pub.get_subscription_count() > 0)
        self._spin_for(1.0)

        # A Goal arriving before the first accepted map is retained and queued
        # automatically when that map arrives.
        pending_checkpoint = len(self.paths)
        self.goal_pub.publish(self._goal())
        self._spin_for(0.25)
        self.assertEqual(len(self.paths), pending_checkpoint)
        self.cloud_pub.publish(self._cloud('flat'))
        self._wait(lambda: (
            len(self.paths) > pending_checkpoint
            and len(self.paths[-1].poses) > 0))
        flat = self.paths[-1]
        proc_output.assertWaitFor(
            expected_output='stored Goal as pending', process=planner,
            timeout=5.0)
        self.assertEqual(flat.header.frame_id, MAP)
        self.assertAlmostEqual(flat.poses[0].pose.position.x, -0.60, places=5)
        self.assertAlmostEqual(flat.poses[-1].pose.position.x, 0.60, places=5)
        final = flat.poses[-1].pose.orientation
        self.assertAlmostEqual(math.hypot(final.z, final.w), 1.0, places=6)
        expected = (math.sin(0.35), math.cos(0.35))
        self.assertAlmostEqual(
            abs(final.z * expected[0] + final.w * expected[1]), 1.0, places=6)

        # Both exact endpoint cells are unknown. The bounded resolver projects
        # them onto strict evaluator-valid lattice anchors before graph build.
        query_checkpoint = len(self.paths)
        marker_checkpoint = len(self.query_snaps)
        self.cloud_pub.publish(self._cloud('query_holes'))
        self._spin_for(0.25)
        snapped_goal_yaw = 1.1
        self.goal_pub.publish(self._goal(yaw=snapped_goal_yaw))
        self._wait(lambda: (
            len(self.paths) > query_checkpoint
            and len(self.paths[-1].poses) > 0
            and any(
                marker.type == Marker.LINE_LIST and len(marker.points) == 4
                for output in self.query_snaps[marker_checkpoint:]
                for marker in output.markers)))
        snapped = self.paths[-1]
        self.assertNotAlmostEqual(
            snapped.poses[0].pose.position.x, -0.60, places=5)
        self.assertNotAlmostEqual(
            snapped.poses[-1].pose.position.x, 0.60, places=5)
        self.assertLess(math.hypot(
            snapped.poses[0].pose.position.x + 0.60,
            snapped.poses[0].pose.position.y), 0.30 + 1.0e-6)
        self.assertLess(math.hypot(
            snapped.poses[-1].pose.position.x - 0.60,
            snapped.poses[-1].pose.position.y), 0.25 + 1.0e-6)
        snapped_orientation = snapped.poses[-1].pose.orientation
        self.assertAlmostEqual(
            abs(
                snapped_orientation.z * math.sin(0.5 * snapped_goal_yaw)
                + snapped_orientation.w * math.cos(0.5 * snapped_goal_yaw)),
            1.0, places=6)
        self.assertTrue(any(
            marker.type == Marker.LINE_LIST and len(marker.points) == 4
            for output in self.query_snaps[marker_checkpoint:]
            for marker in output.markers))
        proc_output.assertWaitFor(
            expected_output='start_snapped=true', process=planner, timeout=5.0)
        proc_output.assertWaitFor(
            expected_output='goal_snapped=true', process=planner, timeout=5.0)

        # A query with no strict-valid cell inside the fixed radius fails before
        # graph construction; the diagnostic summary therefore has zero nodes.
        failed_query_checkpoint = len(self.paths)
        self.goal_pub.publish(self._goal(x=5.0, yaw=0.4))
        self._wait(lambda: (
            len(self.paths) > failed_query_checkpoint
            and len(self.paths[-1].poses) == 0))
        proc_output.assertWaitFor(
            expected_output='Goal query resolution failed:',
            process=planner, timeout=5.0)
        proc_output.assertWaitFor(
            expected_output=(
                'message=goal_query_resolution_failed nodes=0 edges=0'),
            process=planner, timeout=5.0)

        # Restore the legacy exact-valid fixture for lifecycle tests below.
        flat = self._publish_and_wait_path('flat', True)
        self.assertAlmostEqual(flat.poses[0].pose.position.x, -0.60, places=5)
        self.assertAlmostEqual(flat.poses[-1].pose.position.x, 0.60, places=5)

        # Same-frame updates outside the remaining corridor, cost-only changes,
        # and identical hashes retain the active Path without republishing it.
        retained_count = len(self.paths)
        self.cloud_pub.publish(self._cloud('outside'))
        self._spin_for(0.25)
        self.assertEqual(len(self.paths), retained_count)
        self.cloud_pub.publish(self._cloud('outside'))
        self._spin_for(0.25)
        self.assertEqual(len(self.paths), retained_count)
        self.cloud_pub.publish(self._cloud('crossable'))
        self._spin_for(0.25)
        self.assertEqual(len(self.paths), retained_count)

        # The first terrain-derived invalid snapshot stops public motion but
        # retains the internal Path. Two valid confirmations recover it.
        suspended_count = len(self.paths)
        self.cloud_pub.publish(self._cloud('unknown_a'))
        self._wait(lambda: (
            len(self.paths) > suspended_count
            and len(self.paths[-1].poses) == 0
            and self.failures
            and self.failures[-1].action == Marker.ADD
        ))
        stopped_count = len(self.paths)
        self.cloud_pub.publish(self._cloud('outside'))
        self._spin_for(0.25)
        self.assertEqual(len(self.paths), stopped_count)
        self.cloud_pub.publish(self._cloud('flat'))
        self._wait(lambda: (
            len(self.paths) > stopped_count
            and len(self.paths[-1].poses) > 0
            and self.failures[-1].action == Marker.DELETE
        ))
        proc_output.assertWaitFor(
            expected_output='--PATH_RECOVERED[', process=planner,
            timeout=5.0)

        # Two invalid snapshots confirm deletion. The first automatic plan on
        # the invalid map fails; a newer flat map plus the retry period starts
        # a bounded retry and returns to TRACKING.
        confirmed_count = len(self.paths)
        self.cloud_pub.publish(self._cloud('unknown_a'))
        self._wait(lambda: (
            len(self.paths) > confirmed_count
            and len(self.paths[-1].poses) == 0))
        self.cloud_pub.publish(self._cloud('unknown_b'))
        proc_output.assertWaitFor(
            expected_output='--PATH_INVALID_CONFIRMED[', process=planner,
            timeout=5.0)
        proc_output.assertWaitFor(
            expected_output='--PLAN_FAILED[replan_failed]--> WAITING_RETRY',
            process=planner, timeout=10.0)
        self.cloud_pub.publish(self._cloud('flat'))
        self._wait(lambda: (
            len(self.paths) > confirmed_count
            and len(self.paths[-1].poses) > 0), timeout=15.0)
        proc_output.assertWaitFor(
            expected_output='--RETRY_READY[', process=planner,
            timeout=5.0)

        # Restore an active path for the remaining independent contracts.
        flat = self._publish_and_wait_path('flat', True)
        self.assertGreater(len(flat.poses), 0)

        invalidated_count = len(self.paths)
        self.cloud_pub.publish(self._cloud('over'))
        self._wait(lambda: len(self.paths) > invalidated_count)
        self.assertEqual(len(self.paths[-1].poses), 0)
        flat = self._publish_and_wait_path('flat', True)

        crossable = self._publish_and_wait_path('crossable', True)
        self.assertGreater(len(crossable.poses), 0)
        proc_output.assertWaitFor(
            expected_output='path_height_event_count=', process=planner,
            timeout=5.0)
        proc_output.assertWaitFor(
            expected_output='core_total_time_ms=', process=planner,
            timeout=5.0)

        over = self._publish_and_wait_path('over', False)
        self.assertEqual(len(over.poses), 0)
        unknown = self._publish_and_wait_path('unknown', False)
        self.assertEqual(len(unknown.poses), 0)

        late_node = rclpy.create_node('step_wavefront_late_subscriber')
        received = []
        latched = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        subscription = late_node.create_subscription(
            Path, PATH, received.append, latched)
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline and not received:
            rclpy.spin_once(late_node, timeout_sec=0.05)
        self.assertTrue(received)
        self.assertEqual(len(received[-1].poses), 0)
        late_node.destroy_subscription(subscription)
        late_node.destroy_node()

        malformed_checkpoint = (
            len(self.paths), len(self.nodes), len(self.edges), len(self.rejected))
        malformed = self._goal()
        malformed.pose.orientation.z = 0.0
        malformed.pose.orientation.w = 0.0
        self.goal_pub.publish(malformed)
        self._assert_full_reset(malformed_checkpoint, MAP)

        frame_checkpoint = (
            len(self.paths), len(self.nodes), len(self.edges), len(self.rejected))
        other_frame_cloud = self._cloud('flat')
        other_frame_cloud.header.frame_id = 'other_map'
        self.cloud_pub.publish(other_frame_cloud)
        self._assert_full_reset(frame_checkpoint, 'other_map')


@launch_testing.post_shutdown_test()
class TestProcesses(unittest.TestCase):

    def test_exit(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
