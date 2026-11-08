"""Synthetic pose tests on an isolated ROS domain; never uses /cmd_vel."""
import math
import time
import unittest

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import pytest
import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from nav_msgs.msg import Path
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from tf2_ros import TransformBroadcaster


@pytest.mark.launch_test
def generate_test_description():
    nodes = []
    for label, enabled in [('oriented', True), ('legacy', False)]:
        nodes.append(launch_ros.actions.Node(
            package='pongbot_navigation', executable='simple_pure_pursuit_controller',
            name='goal_test_' + label, output='screen', parameters=[{
                'path_topic': '/goal_test/path', 'cmd_vel_topic': '/goal_test/' + label,
                'base_frame': 'goal_test_base', 'enforce_goal_orientation': enabled,
                'goal_tolerance_m': 0.20, 'goal_yaw_tolerance_rad': math.radians(5),
                'goal_max_angular_velocity_rps': 0.30,
            }]))
    return launch.LaunchDescription(nodes + [launch_testing.actions.ReadyToTest()])


class TestGoalOrientation(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('goal_test_driver')
        cls.tf = TransformBroadcaster(cls.node)
        cls.robot = (0., 0., 0.)
        cls.commands = {'oriented': [], 'legacy': []}
        cls.subs = [cls.node.create_subscription(
            Twist, '/goal_test/' + k, lambda m, k=k: cls.commands[k].append(m), 50)
            for k in cls.commands]
        cls.pub = cls.node.create_publisher(Path, '/goal_test/path', QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL))

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def spin(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            t = TransformStamped()
            t.header.frame_id = 'map'
            t.child_frame_id = 'goal_test_base'
            t.header.stamp = self.node.get_clock().now().to_msg()
            t.transform.translation.x, t.transform.translation.y = self.robot[:2]
            t.transform.rotation.z = math.sin(self.robot[2] / 2)
            t.transform.rotation.w = math.cos(self.robot[2] / 2)
            self.tf.sendTransform(t)
            rclpy.spin_once(self.node, timeout_sec=.02)

    def send(self, x, y, yaw, invalid=False):
        path = Path()
        path.header.frame_id = 'map'
        for px, py in [(0., 0.), (x, y)]:
            p = PoseStamped()
            p.header.frame_id = 'map'
            p.pose.position.x, p.pose.position.y = px, py
            if not invalid:
                p.pose.orientation.z = math.sin(yaw / 2)
                p.pose.orientation.w = math.cos(yaw / 2)
            else:
                # geometry_msgs/Quaternion defaults to w=1 in this ROS build.
                p.pose.orientation.w = 0.0
            path.poses.append(p)
        self.pub.publish(path)
        self.spin(.4)  # allow Path and TF callbacks to arrive
        for values in self.commands.values():
            values.clear()
        self.spin(.4)
        self.assertTrue(all(self.commands.values()))

    def test_goal_pose_contract(self):
        deadline = time.monotonic() + 8
        while self.pub.get_subscription_count() < 2 and time.monotonic() < deadline:
            self.spin(.1)
        self.assertEqual(self.pub.get_subscription_count(), 2)
        self.spin(.2)
        self.send(.1, 0., math.pi / 2)
        self.assertTrue(all(c.linear.x == 0 and .05 < c.angular.z <= .30
                            for c in self.commands['oriented']))
        self.assertTrue(all(c.linear.x == 0 and c.angular.z == 0
                            for c in self.commands['legacy']))

        # A replacement target must reverse the final turn.
        self.send(.1, 0., -math.pi / 2)
        self.assertTrue(all(-.30 <= c.angular.z < -.05
                            for c in self.commands['oriented']))

        # Use the short rotation across the +/-pi boundary (+11 degrees).
        self.robot = (0., 0., math.radians(179))
        self.spin(.2)
        self.send(.1, 0., math.radians(-170))
        self.assertTrue(all(.02 < c.angular.z < .30
                            for c in self.commands['oriented']))

        # Within both tolerances completes; 2-degree wrapped yaw error is small.
        self.send(.1, 0., math.radians(-179))
        self.assertTrue(all(c.linear.x == 0 and c.angular.z == 0
                            for c in self.commands['oriented']))

        # Leaving the position region during alignment resumes path tracking.
        self.robot = (0., 0., 0.)
        self.spin(.2)
        self.send(.1, 0., math.pi / 2)
        self.robot = (-.3, 0., 0.)
        self.spin(.4)
        self.commands['oriented'].clear()
        self.spin(.4)
        self.assertTrue(any(c.linear.x > 0 for c in self.commands['oriented']))

        # Matching yaw alone must not complete a distant goal.
        self.send(.8, 0., 0.)
        self.assertTrue(any(c.linear.x > 0 for c in self.commands['oriented']))
        self.send(.1, 0., 0., invalid=True)
        self.assertTrue(all(c.linear.x == 0 and c.angular.z == 0
                            for c in self.commands['oriented']))


@launch_testing.post_shutdown_test()
class TestExit(unittest.TestCase):
    def test_exit(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
