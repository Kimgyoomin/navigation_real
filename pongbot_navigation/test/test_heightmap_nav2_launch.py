#!/usr/bin/env python3

import math
from pathlib import Path
import time
import unittest

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
from nav2_msgs.action import ComputePathToPose
from nav_msgs.msg import Path as NavPath
import pytest
import rclpy
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header


PACKAGE = Path(__file__).parents[1]
CLOUD_TOPIC = '/fastdem/mapping/cloud_global'


@pytest.mark.launch_test
def generate_test_description():
    base = str(PACKAGE / 'config/nav2_rubi_dwb.yaml')
    overlay = str(
        PACKAGE / 'config/nav2_rubi_heightmap_wavefront_overlay.yaml')
    map_yaml = str(PACKAGE / 'maps/RUBI_occupancy_map.yaml')
    static_tf = launch_ros.actions.Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='heightmap_nav2_test_tf', output='screen', arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--frame-id', 'map', '--child-frame-id', 'base_link'])
    map_server = launch_ros.actions.Node(
        package='nav2_map_server', executable='map_server',
        name='map_server', output='screen', parameters=[{
            'use_sim_time': False, 'yaml_filename': map_yaml,
            'topic_name': 'map', 'frame_id': 'map'}])
    planner_server = launch_ros.actions.Node(
        package='nav2_planner', executable='planner_server',
        name='planner_server', output='screen', parameters=[base, overlay])
    manager = launch_ros.actions.Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='heightmap_nav2_test_manager', output='screen', parameters=[{
            'use_sim_time': False, 'autostart': True,
            'node_names': ['map_server', 'planner_server']}])
    return (
        launch.LaunchDescription([
            static_tf, map_server, planner_server, manager,
            launch_testing.actions.ReadyToTest(),
        ]),
        {'planner_server': planner_server},
    )


class TestHeightmapPlannerServer(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('heightmap_nav2_launch_test_client')
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        cls.cloud_pub = cls.node.create_publisher(
            PointCloud2, CLOUD_TOPIC, qos)
        cls.plans = []
        cls.plan_sub = cls.node.create_subscription(
            NavPath, '/plan', cls.plans.append, 10)
        cls.client = ActionClient(
            cls.node, ComputePathToPose, '/compute_path_to_pose')

    @classmethod
    def tearDownClass(cls):
        cls.client.destroy()
        cls.node.destroy_node()
        rclpy.shutdown()

    def _spin_until(self, predicate, timeout=20.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if predicate():
                return
            rclpy.spin_once(self.node, timeout_sec=0.05)
        self.assertTrue(predicate(), 'timed out waiting for Nav2 integration')

    @staticmethod
    def _cloud(kind='flat'):
        points = []
        for y in range(-24, 25):
            for x in range(-34, 35):
                if kind == 'unknown' and x == 0:
                    continue
                z = 0.081 if kind == 'step' and x >= 0 else 0.0
                points.append((0.05 * x, 0.05 * y, z))
        return point_cloud2.create_cloud_xyz32(Header(frame_id='map'), points)

    def _publish_cloud(self, kind='flat'):
        self._spin_until(lambda: self.cloud_pub.get_subscription_count() > 0)
        self.cloud_pub.publish(self._cloud(kind))
        for _ in range(5):
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def _request(self, start_xy=(-0.6, 0.0), goal_xy=(0.6, 0.0),
                 frame='map', quaternion=(0.0, 1.0)):
        request = ComputePathToPose.Goal()
        request.use_start = True
        request.planner_id = 'HeightmapWavefront'
        request.start = PoseStamped()
        request.start.header.frame_id = frame
        request.start.pose.position.x = start_xy[0]
        request.start.pose.position.y = start_xy[1]
        request.start.pose.orientation.w = 1.0
        request.goal = PoseStamped()
        request.goal.header.frame_id = frame
        request.goal.pose.position.x = goal_xy[0]
        request.goal.pose.position.y = goal_xy[1]
        request.goal.pose.orientation.z = quaternion[0]
        request.goal.pose.orientation.w = quaternion[1]
        started = time.monotonic()
        send = self.client.send_goal_async(request)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=10.0)
        self.assertTrue(send.done())
        handle = send.result()
        self.assertTrue(handle.accepted)
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(
            self.node, result_future, timeout_sec=20.0)
        self.assertTrue(result_future.done())
        return result_future.result(), 1000.0 * (time.monotonic() - started)

    def test_planner_server_action_and_performance(
        self, proc_info, proc_output, planner_server
    ):
        proc_info.assertWaitForStartup(process=planner_server, timeout=10.0)
        self.assertTrue(self.client.wait_for_server(timeout_sec=30.0))

        no_map, _ = self._request()
        self.assertEqual(no_map.status, GoalStatus.STATUS_ABORTED)

        self._publish_cloud('flat')
        valid, _ = self._request(quaternion=(3.0 * math.sin(0.35),
                                             3.0 * math.cos(0.35)))
        self.assertEqual(valid.status, GoalStatus.STATUS_SUCCEEDED)
        path = valid.result.path
        self.assertGreater(len(path.poses), 1)
        self.assertEqual(path.header.frame_id, 'map')
        self.assertAlmostEqual(path.poses[0].pose.position.x, -0.6, places=6)
        self.assertAlmostEqual(path.poses[-1].pose.position.x, 0.6, places=6)
        self.assertTrue(all(p.pose.position.z == 0.0 for p in path.poses))
        final = path.poses[-1].pose.orientation
        self.assertAlmostEqual(math.hypot(final.z, final.w), 1.0, places=6)
        self._spin_until(lambda: self.plans and self.plans[-1].poses)

        invalid_quaternion, _ = self._request(quaternion=(0.0, 0.0))
        self.assertEqual(invalid_quaternion.status, GoalStatus.STATUS_ABORTED)
        frame_mismatch, _ = self._request(frame='other_map')
        self.assertEqual(frame_mismatch.status, GoalStatus.STATUS_ABORTED)

        malformed = self._cloud('flat')
        malformed.fields.pop()
        self.cloud_pub.publish(malformed)
        retained, _ = self._request()
        self.assertEqual(retained.status, GoalStatus.STATUS_SUCCEEDED)

        durations = []
        for index in range(30):
            if index % 5 == 0:
                self._publish_cloud('flat')
            y = 0.20 * math.sin(index * 0.4)
            result, elapsed = self._request(
                start_xy=(-0.6, 0.0), goal_xy=(0.6, y))
            self.assertEqual(result.status, GoalStatus.STATUS_SUCCEEDED)
            durations.append(elapsed)
        p95 = sorted(durations)[math.ceil(0.95 * len(durations)) - 1]
        print(f'CREATE_PLAN_E2E_30_P95_MS={p95:.3f}')
        self.assertLess(p95, 1000.0)
        proc_output.assertWaitFor(
            expected_output='create_plan_ms=', process=planner_server,
            timeout=5.0)

        self._publish_cloud('unknown')
        unknown, _ = self._request()
        self.assertEqual(unknown.status, GoalStatus.STATUS_ABORTED)
        self._publish_cloud('step')
        step, _ = self._request()
        self.assertEqual(step.status, GoalStatus.STATUS_ABORTED)


@launch_testing.post_shutdown_test()
class TestProcesses(unittest.TestCase):

    def test_exit(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
