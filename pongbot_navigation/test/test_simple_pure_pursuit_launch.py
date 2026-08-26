#!/usr/bin/env python3

import math
from pathlib import Path
import time
import unittest

from geometry_msgs.msg import Twist
import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
from nav_msgs.msg import Path as NavPath
import pytest
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from visualization_msgs.msg import Marker


PACKAGE = Path(__file__).parents[1]


def _controller(name, base_frame, cmd_topic):
    return launch_ros.actions.Node(
        package='pongbot_navigation',
        executable='simple_pure_pursuit_controller',
        name=name,
        output='screen',
        parameters=[{
            'path_topic': '/test/path',
            'cmd_vel_topic': cmd_topic,
            'map_frame': 'map',
            'base_frame': base_frame,
            'control_frequency_hz': 20.0,
            'transform_timeout_s': 0.01,
            'lookahead_distance_m': 0.35,
            'nominal_linear_velocity_mps': 0.20,
            'goal_tolerance_m': 0.20,
            'max_cross_track_error_m': 0.40,
        }],
    )


@pytest.mark.launch_test
def generate_test_description():
    controller = _controller('simple_pp_test', 'base_link', '/test/cmd_vel')
    missing_tf_controller = _controller(
        'simple_pp_missing_tf_test', 'missing_base_link',
        '/test/missing_tf_cmd_vel')
    static_tf = launch_ros.actions.Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='simple_pp_test_tf', output='screen', arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--frame-id', 'map', '--child-frame-id', 'base_link'])
    return (
        launch.LaunchDescription([
            static_tf, controller, missing_tf_controller,
            launch_testing.actions.ReadyToTest(),
        ]),
        {'controller': controller, 'missing_tf_controller': missing_tf_controller},
    )


class TestSimplePurePursuit(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('simple_pp_launch_test_client')
        path_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        cls.path_pub = cls.node.create_publisher(NavPath, '/test/path', path_qos)
        cls.commands = []
        cls.all_commands = []
        cls.missing_tf_commands = []
        cls.markers = []
        def record_command(command):
            cls.commands.append(command)
            cls.all_commands.append(command)
        cls.node.create_subscription(
            Twist, '/test/cmd_vel', record_command, 20)
        cls.node.create_subscription(
            Twist, '/test/missing_tf_cmd_vel',
            cls.missing_tf_commands.append, 20)
        cls.node.create_subscription(
            Marker, '/rubi/simple_pp/lookahead', cls.markers.append, 10)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def _spin_until(self, predicate, timeout=5.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if predicate():
                return
        self.assertTrue(predicate(), 'timed out waiting for controller output')

    @staticmethod
    def _path(points, frame='map'):
        path = NavPath()
        path.header.frame_id = frame
        path.header.stamp.sec = 1
        for x, y in points:
            pose = path.poses.add() if hasattr(path.poses, 'add') else None
            if pose is None:
                from geometry_msgs.msg import PoseStamped
                pose = PoseStamped()
                path.poses.append(pose)
            pose.header.frame_id = frame
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.w = 1.0
        return path

    def _publish_and_wait(self, path, predicate):
        self.commands.clear()
        self.path_pub.publish(path)
        self._spin_until(lambda: any(predicate(command) for command in self.commands))

    def test_tracking_and_safety_contracts(
        self, proc_info, controller, missing_tf_controller
    ):
        proc_info.assertWaitForStartup(process=controller, timeout=10.0)
        proc_info.assertWaitForStartup(process=missing_tf_controller, timeout=10.0)
        self._spin_until(lambda: self.path_pub.get_subscription_count() == 2)

        straight = self._path([(0.0, 0.0), (0.4, 0.0), (0.8, 0.0)])
        self._publish_and_wait(
            straight,
            lambda cmd: cmd.linear.x > 0.0 and abs(cmd.angular.z) < 1.0e-6,
        )
        nonzero = [cmd for cmd in self.commands if cmd.linear.x > 0.0]
        self.assertTrue(nonzero)
        self.assertLessEqual(max(cmd.linear.x for cmd in nonzero), 0.70)
        self.assertTrue(all(cmd.linear.y == 0.0 for cmd in self.commands))
        self._spin_until(lambda: len(self.markers) > 0)

        curved = self._path([
            (0.0, 0.0), (0.2, 0.0), (0.4, 0.2), (0.6, 0.4)])
        self._publish_and_wait(curved, lambda cmd: cmd.angular.z > 0.0)
        self.assertLessEqual(
            max(abs(cmd.angular.z) for cmd in self.commands), 1.50)

        # A replacement straight Path resets progress and resumes tracking.
        self._publish_and_wait(straight, lambda cmd: cmd.linear.x > 0.0)

        empty = self._path([])
        self._publish_and_wait(
            empty,
            lambda cmd: cmd.linear.x == 0.0 and cmd.angular.z == 0.0,
        )

        near_goal = self._path([(0.0, 0.0), (0.1, 0.0)])
        self._publish_and_wait(
            near_goal,
            lambda cmd: cmd.linear.x == 0.0 and cmd.angular.z == 0.0,
        )

        cross_track = self._path([(0.0, 0.5), (0.8, 0.5)])
        self._publish_and_wait(
            cross_track,
            lambda cmd: cmd.linear.x == 0.0 and cmd.angular.z == 0.0,
        )

        nonfinite = self._path([(0.0, 0.0), (math.nan, 0.0)])
        self._publish_and_wait(
            nonfinite,
            lambda cmd: cmd.linear.x == 0.0 and cmd.angular.z == 0.0,
        )

        # The parallel controller has an unavailable base frame and must never
        # retain or synthesize a non-zero command for the same valid Path.
        self.missing_tf_commands.clear()
        self.path_pub.publish(straight)
        self._spin_until(lambda: len(self.missing_tf_commands) > 0)
        self.assertTrue(all(
            cmd.linear.x == 0.0 and cmd.linear.y == 0.0 and
            cmd.angular.z == 0.0
            for cmd in self.missing_tf_commands
        ))
        print(
            'SIMPLE_PP_MAX_OBSERVED '
            f'linear_x={max(cmd.linear.x for cmd in self.all_commands):.6f} '
            f'abs_angular_z={max(abs(cmd.angular.z) for cmd in self.all_commands):.6f}'
        )


@launch_testing.post_shutdown_test()
class TestProcesses(unittest.TestCase):

    def test_exit(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
