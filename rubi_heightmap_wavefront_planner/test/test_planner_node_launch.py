#!/usr/bin/env python3

# Copyright 2026
# Licensed under the Apache License, Version 2.0

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
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker, MarkerArray


MAP_FRAME = 'wavefront_launch_test_map'
BASE_FRAME = 'wavefront_launch_test_body'
INPUT_CLOUD_TOPIC = '/wavefront_launch_test/fastdem_cloud'
GOAL_TOPIC = '/wavefront_launch_test/goal'
PATH_TOPIC = '/wavefront_launch_test/path'
NODES_TOPIC = '/wavefront_launch_test/nodes'
EDGES_TOPIC = '/wavefront_launch_test/edges'
REJECTED_TOPIC = '/wavefront_launch_test/rejected'


def _planner_parameters():
    return {
        'use_sim_time': False,
        'planner_mode': 'wavefront',
        'input_cloud_topic': INPUT_CLOUD_TOPIC,
        'goal_topic': GOAL_TOPIC,
        'path_topic': PATH_TOPIC,
        'debug_nodes_topic': NODES_TOPIC,
        'debug_edges_topic': EDGES_TOPIC,
        'debug_rejected_topic': REJECTED_TOPIC,
        'base_frame': BASE_FRAME,
        'map_resolution_m': 0.05,
        'lattice_tolerance_m': 0.01,
        'reject_duplicate_cells': True,
        'max_grid_cells': 100000,
        'transform_timeout_s': 1.0,
        'pca_analysis_radius_m': 0.16,
        'pca_min_points': 5,
        # A zero-radius footprint isolates the edge hard gate in the barrier
        # scenario: a proposal endpoint across the unknown column remains a
        # valid node, while the source-to-candidate edge crosses unknown cells.
        'support_radius_m': 0.0,
        'minimum_observed_support_ratio': 1.0,
        'max_slope_deg': 15.0,
        'max_roughness_m': -1.0,
        'max_step_height_m': 0.08,
        'edge_check_spacing_m': 0.025,
        'check_footprint_along_edge': True,
        'node_sampling_distance_m': 0.30,
        'samples_per_expansion': 20,
        'merge_radius_m': 0.20,
        'neighbor_connection_radius_m': 0.45,
        'goal_connection_distance_m': 0.45,
        'max_nodes': 1000,
        'max_expansions': 500,
        'max_build_time_ms': 5000,
        'stop_when_goal_connected': True,
        'distance_weight': 1.0,
        'slope_risk_weight': 3.0,
        'step_risk_weight': 0.0,
        'roughness_risk_weight': 0.0,
        'path_output_spacing_m': 0.05,
        'node_marker_scale_m': 0.08,
        'edge_marker_width_m': 0.025,
        'path_marker_width_m': 0.08,
        'rejected_marker_scale_m': 0.07,
        'max_rejected_markers': 5000,
    }


@pytest.mark.launch_test
def generate_test_description():
    static_tf = launch_ros.actions.Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='wavefront_launch_test_static_tf',
        arguments=[
            '--x',
            '-0.75',
            '--y',
            '0.0',
            '--z',
            '0.0',
            '--frame-id',
            MAP_FRAME,
            '--child-frame-id',
            BASE_FRAME,
        ],
        output='screen',
    )
    planner = launch_ros.actions.Node(
        package='rubi_heightmap_wavefront_planner',
        executable='wavefront_planner_node',
        name='wavefront_planner_launch_test',
        parameters=[_planner_parameters()],
        output='screen',
    )
    return (
        launch.LaunchDescription(
            [
                static_tf,
                planner,
                launch_testing.actions.ReadyToTest(),
            ]
        ),
        {'planner': planner, 'static_tf': static_tf},
    )


class TestPlannerNodeContract(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('wavefront_planner_contract_test_client')

        volatile_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        latched_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        cls.cloud_publisher = cls.node.create_publisher(
            PointCloud2, INPUT_CLOUD_TOPIC, volatile_qos
        )
        cls.goal_publisher = cls.node.create_publisher(
            PoseStamped, GOAL_TOPIC, volatile_qos
        )
        cls.received = {
            'path': [],
            'nodes': [],
            'edges': [],
            'rejected': [],
        }
        cls.subscriptions = [
            cls.node.create_subscription(
                Path,
                PATH_TOPIC,
                lambda message: cls.received['path'].append(message),
                latched_qos,
            ),
            cls.node.create_subscription(
                MarkerArray,
                NODES_TOPIC,
                lambda message: cls.received['nodes'].append(message),
                latched_qos,
            ),
            cls.node.create_subscription(
                MarkerArray,
                EDGES_TOPIC,
                lambda message: cls.received['edges'].append(message),
                latched_qos,
            ),
            cls.node.create_subscription(
                MarkerArray,
                REJECTED_TOPIC,
                lambda message: cls.received['rejected'].append(message),
                latched_qos,
            ),
        ]

        cls.tf_buffer = Buffer()
        cls.tf_listener = TransformListener(
            cls.tf_buffer, cls.node, spin_thread=False
        )

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def _spin_until(self, predicate, timeout, failure_message):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if predicate():
                return
            rclpy.spin_once(self.node, timeout_sec=0.05)
        self.assertTrue(predicate(), failure_message)

    def _spin_for(self, duration):
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            rclpy.spin_once(
                self.node,
                timeout_sec=min(0.05, max(0.0, deadline - time.monotonic())),
            )

    def _checkpoint(self):
        return {
            name: len(messages)
            for name, messages in self.received.items()
        }

    def _wait_for_snapshot(self, checkpoint, timeout=15.0):
        self._spin_until(
            lambda: all(
                len(self.received[name]) > count
                for name, count in checkpoint.items()
            ),
            timeout,
            'Timed out waiting for a complete path/node/edge/rejected snapshot',
        )
        return {
            name: self.received[name][-1]
            for name in checkpoint
        }

    def _assert_same_snapshot_stamp(self, snapshot):
        path = snapshot['path']
        expected_stamp = (path.header.stamp.sec, path.header.stamp.nanosec)
        self.assertEqual(MAP_FRAME, path.header.frame_id)
        for name in ('nodes', 'edges', 'rejected'):
            markers = snapshot[name].markers
            self.assertTrue(markers, name)
            for marker in markers:
                marker_stamp = (
                    marker.header.stamp.sec,
                    marker.header.stamp.nanosec,
                )
                self.assertEqual(expected_stamp, marker_stamp, name)
                self.assertEqual(MAP_FRAME, marker.header.frame_id, name)

    @staticmethod
    def _add_markers(marker_array, marker_type):
        return [
            marker
            for marker in marker_array.markers
            if marker.action == Marker.ADD and marker.type == marker_type
        ]

    @staticmethod
    def _marker_colors(marker):
        return list(marker.colors) if marker.colors else [marker.color]

    @classmethod
    def _marker_has_color(cls, marker, predicate):
        return any(predicate(color) for color in cls._marker_colors(marker))

    @staticmethod
    def _is_green(color):
        return color.r < 0.35 and color.g > 0.65 and color.b < 0.45

    @staticmethod
    def _is_red(color):
        return color.r > 0.75 and color.g < 0.25 and color.b < 0.25

    @staticmethod
    def _is_yellow(color):
        return color.r > 0.75 and color.g > 0.75 and color.b < 0.25

    @staticmethod
    def _make_cloud(with_barrier, corner_z=0.0):
        points = []
        for iy in range(-18, 19):
            for ix in range(-24, 25):
                if with_barrier and ix == 0:
                    continue
                z = corner_z if (ix, iy) == (-24, -18) else 0.0
                points.append((0.05 * ix, 0.05 * iy, z))
        header = Header()
        header.frame_id = MAP_FRAME
        return point_cloud2.create_cloud_xyz32(header, points)

    def _make_goal(self, x, y=0.0):
        goal = PoseStamped()
        goal.header.frame_id = MAP_FRAME
        goal.header.stamp = self.node.get_clock().now().to_msg()
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.orientation.w = 1.0
        return goal

    def _assert_success_snapshot(self, snapshot):
        self._assert_same_snapshot_stamp(snapshot)
        path = snapshot['path']
        self.assertEqual(MAP_FRAME, path.header.frame_id)
        self.assertGreater(len(path.poses), 0)
        self.assertAlmostEqual(-0.75, path.poses[0].pose.position.x, places=6)
        self.assertAlmostEqual(0.0, path.poses[0].pose.position.y, places=6)
        self.assertAlmostEqual(0.75, path.poses[-1].pose.position.x, places=6)
        self.assertAlmostEqual(0.0, path.poses[-1].pose.position.y, places=6)
        path_stamp = (path.header.stamp.sec, path.header.stamp.nanosec)
        for index, pose in enumerate(path.poses):
            values = (
                pose.pose.position.x,
                pose.pose.position.y,
                pose.pose.position.z,
                pose.pose.orientation.x,
                pose.pose.orientation.y,
                pose.pose.orientation.z,
                pose.pose.orientation.w,
            )
            self.assertTrue(all(math.isfinite(value) for value in values))
            self.assertEqual(MAP_FRAME, pose.header.frame_id)
            self.assertEqual(
                path_stamp,
                (pose.header.stamp.sec, pose.header.stamp.nanosec),
            )
            self.assertAlmostEqual(0.0, pose.pose.position.z, places=6)
            self.assertAlmostEqual(0.0, pose.pose.orientation.x, places=6)
            self.assertAlmostEqual(0.0, pose.pose.orientation.y, places=6)
            quaternion_norm = math.sqrt(
                pose.pose.orientation.x ** 2
                + pose.pose.orientation.y ** 2
                + pose.pose.orientation.z ** 2
                + pose.pose.orientation.w ** 2
            )
            self.assertAlmostEqual(1.0, quaternion_norm, places=6)

            if len(path.poses) > 1:
                tangent_from = (
                    pose if index + 1 < len(path.poses)
                    else path.poses[index - 1]
                )
                tangent_to = (
                    path.poses[index + 1]
                    if index + 1 < len(path.poses)
                    else pose
                )
                expected_yaw = math.atan2(
                    tangent_to.pose.position.y
                    - tangent_from.pose.position.y,
                    tangent_to.pose.position.x
                    - tangent_from.pose.position.x,
                )
            else:
                expected_yaw = 0.0
            expected_z = math.sin(0.5 * expected_yaw)
            expected_w = math.cos(0.5 * expected_yaw)
            orientation_dot = abs(
                pose.pose.orientation.z * expected_z
                + pose.pose.orientation.w * expected_w
            )
            self.assertAlmostEqual(1.0, orientation_dot, places=6)

            if index > 0:
                previous = path.poses[index - 1].pose.position
                spacing = math.hypot(
                    pose.pose.position.x - previous.x,
                    pose.pose.position.y - previous.y,
                )
                self.assertLessEqual(spacing, 0.05 + 1.0e-8)

        node_markers = self._add_markers(
            snapshot['nodes'], Marker.SPHERE_LIST
        )
        self.assertGreater(sum(len(marker.points) for marker in node_markers), 2)
        self.assertTrue(
            any(
                self._marker_has_color(marker, self._is_green)
                for marker in node_markers
            ),
            'No green accepted sampled node was published',
        )

        edge_markers = self._add_markers(
            snapshot['edges'], Marker.LINE_LIST
        )
        self.assertGreaterEqual(
            sum(len(marker.points) for marker in edge_markers), 2
        )
        self.assertTrue(
            any(
                self._marker_has_color(marker, self._is_green)
                for marker in edge_markers
            ),
            'No green accepted edge was published',
        )

        path_markers = self._add_markers(
            snapshot['edges'], Marker.LINE_STRIP
        )
        self.assertTrue(
            any(
                marker.points
                and self._marker_has_color(marker, self._is_yellow)
                for marker in path_markers
            ),
            'No non-empty yellow final-path marker was published',
        )

    def _assert_clear_snapshot(self, snapshot):
        self._assert_same_snapshot_stamp(snapshot)
        self.assertEqual(0, len(snapshot['path'].poses))
        for name in ('nodes', 'edges', 'rejected'):
            markers = snapshot[name].markers
            self.assertEqual(1, len(markers), name)
            self.assertEqual(Marker.DELETEALL, markers[0].action, name)

    def _assert_partial_failure_snapshot(self, snapshot):
        # An explicit Goal first invalidates the old executable Path with an
        # empty Path-only publication. The ensuing worker failure replaces only
        # the debug snapshot, so its markers intentionally have a newer stamp.
        self.assertEqual(0, len(snapshot['path'].poses))

        node_markers = self._add_markers(
            snapshot['nodes'], Marker.SPHERE_LIST
        )
        edge_markers = self._add_markers(
            snapshot['edges'], Marker.LINE_LIST
        )
        rejected_edges = self._add_markers(
            snapshot['rejected'], Marker.LINE_LIST
        )
        self.assertGreater(
            sum(len(marker.points) for marker in node_markers), 0
        )
        self.assertGreater(
            sum(len(marker.points) for marker in edge_markers), 0
        )
        self.assertTrue(
            any(
                len(marker.points) >= 2
                and len(marker.points) % 2 == 0
                and self._marker_has_color(marker, self._is_red)
                for marker in rejected_edges
            ),
            'Barrier failure did not publish a red source-to-candidate edge',
        )

    def _assert_invalid_goal_snapshot(self, snapshot):
        self._assert_same_snapshot_stamp(snapshot)
        self.assertEqual(0, len(snapshot['path'].poses))
        accepted_node_points = sum(
            len(marker.points)
            for marker in self._add_markers(
                snapshot['nodes'], Marker.SPHERE_LIST
            )
        )
        accepted_edge_points = sum(
            len(marker.points)
            for marker in self._add_markers(
                snapshot['edges'], Marker.LINE_LIST
            )
        )
        self.assertEqual(0, accepted_node_points)
        self.assertEqual(0, accepted_edge_points)

        rejected_nodes = self._add_markers(
            snapshot['rejected'], Marker.SPHERE_LIST
        )
        self.assertTrue(
            any(
                marker.points
                and self._marker_has_color(marker, self._is_red)
                for marker in rejected_nodes
            ),
            'Invalid goal did not publish its red diagnostic point',
        )

    def _assert_late_subscriber_receives_partial_failure(self):
        late_node = rclpy.create_node(
            'wavefront_planner_contract_late_subscriber'
        )
        latched_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        received = {
            'path': [],
            'nodes': [],
            'edges': [],
            'rejected': [],
        }
        subscriptions = [
            late_node.create_subscription(
                Path,
                PATH_TOPIC,
                lambda message: received['path'].append(message),
                latched_qos,
            ),
            late_node.create_subscription(
                MarkerArray,
                NODES_TOPIC,
                lambda message: received['nodes'].append(message),
                latched_qos,
            ),
            late_node.create_subscription(
                MarkerArray,
                EDGES_TOPIC,
                lambda message: received['edges'].append(message),
                latched_qos,
            ),
            late_node.create_subscription(
                MarkerArray,
                REJECTED_TOPIC,
                lambda message: received['rejected'].append(message),
                latched_qos,
            ),
        ]
        try:
            deadline = time.monotonic() + 5.0
            while time.monotonic() < deadline and not all(received.values()):
                rclpy.spin_once(late_node, timeout_sec=0.05)
            self.assertTrue(
                all(received.values()),
                'Late subscriber did not receive every transient-local output',
            )
            snapshot = {
                name: messages[-1]
                for name, messages in received.items()
            }
            self._assert_partial_failure_snapshot(snapshot)
        finally:
            for subscription in subscriptions:
                late_node.destroy_subscription(subscription)
            late_node.destroy_node()

    def test_planner_node_publication_and_map_update_contract(
        self, proc_info, proc_output, planner
    ):
        proc_info.assertWaitForStartup(process=planner, timeout=10.0)
        self._spin_until(
            lambda: (
                self.cloud_publisher.get_subscription_count() > 0
                and self.goal_publisher.get_subscription_count() > 0
                and all(
                    self.node.count_publishers(topic) > 0
                    for topic in (
                        PATH_TOPIC,
                        NODES_TOPIC,
                        EDGES_TOPIC,
                        REJECTED_TOPIC,
                    )
                )
            ),
            10.0,
            'Planner ROS interfaces were not discovered',
        )
        self._spin_until(
            lambda: self.tf_buffer.can_transform(
                MAP_FRAME, BASE_FRAME, Time()
            ),
            10.0,
            'Static map-to-base transform was not received',
        )

        flat_cloud = self._make_cloud(with_barrier=False)
        reachable_goal = self._make_goal(0.75)

        # A goal sent before the first accepted map is held, then planned once
        # that map arrives.
        self.goal_publisher.publish(reachable_goal)
        proc_output.assertWaitFor(
            expected_output='stored the latest goal as pending',
            process=planner,
            timeout=5.0,
        )
        success_checkpoint = self._checkpoint()
        self.cloud_publisher.publish(flat_cloud)
        success_snapshot = self._wait_for_snapshot(success_checkpoint)
        self._assert_success_snapshot(success_snapshot)
        self._spin_for(0.25)

        # Header stamps are not part of the elevation-content hash. Republishing
        # the exact same lattice must preserve the current latched snapshot.
        identical_checkpoint = self._checkpoint()
        identical_cloud = self._make_cloud(with_barrier=False)
        identical_cloud.header.stamp = self.node.get_clock().now().to_msg()
        self.cloud_publisher.publish(identical_cloud)
        self._spin_for(0.75)
        self.assertEqual(identical_checkpoint, self._checkpoint())

        # A change outside the remaining corridor updates the map generation
        # but does not replace a valid Path or its debug markers.
        outside_change_checkpoint = self._checkpoint()
        self.cloud_publisher.publish(
            self._make_cloud(with_barrier=False, corner_z=0.01)
        )
        self._spin_for(0.75)
        self.assertEqual(
            outside_change_checkpoint,
            self._checkpoint(),
            'Map change outside the corridor modified the retained output',
        )

        # A same-frame material map update changes only the accepted map state
        # at this lifecycle stage. It must preserve the transient-local output
        # snapshot until corridor validation is implemented.
        barrier_cloud = self._make_cloud(with_barrier=True, corner_z=0.01)
        changed_checkpoint = self._checkpoint()
        self.cloud_publisher.publish(barrier_cloud)
        self._spin_for(0.75)
        self.assertEqual(
            changed_checkpoint,
            self._checkpoint(),
            'Same-frame map update modified Path or debug output before validation',
        )

        # The same unknown corridor in a second, distinct map generation is a
        # confirmed soft failure. It invalidates Path once and queues one
        # automatic replan; the barrier makes that replan fail, while the
        # latched Path remains empty rather than being published twice.
        confirmed_checkpoint = self._checkpoint()
        self.cloud_publisher.publish(
            self._make_cloud(with_barrier=True, corner_z=0.02)
        )
        self._spin_until(
            lambda: len(self.received['path']) > confirmed_checkpoint['path'],
            10.0,
            'Confirmed corridor invalidation did not publish empty Path',
        )
        self._spin_for(1.0)
        self.assertEqual(
            confirmed_checkpoint['path'] + 1,
            len(self.received['path']),
            'One invalid episode published empty Path more than once',
        )
        self.assertEqual(0, len(self.received['path'][-1].poses))

        malformed_checkpoint = self._checkpoint()
        malformed = PointCloud2()
        malformed.header.frame_id = MAP_FRAME
        self.cloud_publisher.publish(malformed)
        self._spin_for(0.5)
        self.assertEqual(
            malformed_checkpoint,
            self._checkpoint(),
            'Malformed cloud modified the prior map/output snapshot',
        )

        # The same reachable goal cannot cross the unknown full-height barrier.
        # Core failure must replace the clear with its partial accepted graph and
        # rejected source-to-candidate edges while keeping Path empty.
        failure_checkpoint = self._checkpoint()
        self.goal_publisher.publish(self._make_goal(0.75))
        failure_snapshot = self._wait_for_snapshot(
            failure_checkpoint, timeout=15.0
        )
        self._assert_partial_failure_snapshot(failure_snapshot)

        # All four publishers are reliable + transient-local, including an empty
        # Path representing the latest failed request.
        self._assert_late_subscriber_receives_partial_failure()

        # A goal in the unknown barrier is rejected before accepted graph nodes
        # are created. Publishing an empty graph plus the available diagnostic
        # must be safe and must clear the previous partial snapshot.
        invalid_checkpoint = self._checkpoint()
        self.goal_publisher.publish(self._make_goal(0.0))
        invalid_snapshot = self._wait_for_snapshot(invalid_checkpoint)
        self._assert_invalid_goal_snapshot(invalid_snapshot)

        # A map-frame change is the one map update that performs a full reset.
        frame_change_checkpoint = self._checkpoint()
        frame_change_cloud = self._make_cloud(with_barrier=False)
        frame_change_cloud.header.frame_id = 'wavefront_launch_test_other_map'
        self.cloud_publisher.publish(frame_change_cloud)
        frame_change_snapshot = self._wait_for_snapshot(frame_change_checkpoint)
        self.assertEqual(0, len(frame_change_snapshot['path'].poses))
        for name in ('nodes', 'edges', 'rejected'):
            markers = frame_change_snapshot[name].markers
            self.assertEqual(1, len(markers), name)
            self.assertEqual(Marker.DELETEALL, markers[0].action, name)
            self.assertEqual(
                'wavefront_launch_test_other_map',
                markers[0].header.frame_id,
            )


@launch_testing.post_shutdown_test()
class TestPlannerProcessesExitCleanly(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
