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
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header


MAP_FRAME = 'rrt_star_launch_test_map'
BASE_FRAME = 'rrt_star_launch_test_body'
INPUT_CLOUD_TOPIC = '/rrt_star_launch_test/fastdem_cloud'
GOAL_TOPIC = '/rrt_star_launch_test/goal'
PATH_TOPIC = '/rrt_star_launch_test/path'


def _parameters():
    return {
        'planner_mode': 'rrt_star',
        'input_cloud_topic': INPUT_CLOUD_TOPIC,
        'goal_topic': GOAL_TOPIC,
        'path_topic': PATH_TOPIC,
        'debug_nodes_topic': '/rrt_star_launch_test/nodes',
        'debug_edges_topic': '/rrt_star_launch_test/edges',
        'debug_rejected_topic': '/rrt_star_launch_test/rejected',
        'base_frame': BASE_FRAME,
        'map_resolution_m': 0.05,
        'lattice_tolerance_m': 0.01,
        'pca_analysis_radius_m': 0.30,
        'pca_min_points': 6,
        'support_radius_m': 0.20,
        'minimum_observed_support_ratio': 1.00,
        'max_slope_deg': 15.0,
        'max_step_height_m': 0.08,
        'edge_check_spacing_m': 0.025,
        'check_footprint_along_edge': True,
        'rrt_star.max_iterations': 700,
        'rrt_star.goal_bias': 0.30,
        'rrt_star.max_planning_time_ms': 0,
        'rrt_star.stop_on_first_solution': True,
        'rrt_star.random_seed': 42,
        'path_output_spacing_m': 0.05,
    }


@pytest.mark.launch_test
def generate_test_description():
    static_tf = launch_ros.actions.Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='rrt_star_launch_test_static_tf',
        arguments=[
            '--x', '-0.75', '--y', '0.0', '--z', '0.0',
            '--frame-id', MAP_FRAME, '--child-frame-id', BASE_FRAME,
        ],
    )
    planner = launch_ros.actions.Node(
        package='rubi_heightmap_wavefront_planner',
        executable='wavefront_planner_node',
        name='rrt_star_planner_launch_test',
        parameters=[_parameters()],
        output='screen',
    )
    return launch.LaunchDescription(
        [static_tf, planner, launch_testing.actions.ReadyToTest()]
    ), {'planner': planner}


class TestRrtStarFiveCentimeterLaunch(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('rrt_star_five_centimeter_test_client')
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        path_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        cls.cloud_publisher = cls.node.create_publisher(PointCloud2, INPUT_CLOUD_TOPIC, qos)
        cls.goal_publisher = cls.node.create_publisher(PoseStamped, GOAL_TOPIC, qos)
        cls.paths = []
        cls.path_subscription = cls.node.create_subscription(
            Path, PATH_TOPIC, cls.paths.append, path_qos)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def _spin_until(self, predicate, timeout, message):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if predicate():
                return
            rclpy.spin_once(self.node, timeout_sec=0.05)
        self.assertTrue(predicate(), message)

    @staticmethod
    def _flat_cloud():
        header = Header()
        header.frame_id = MAP_FRAME
        points = [
            (0.05 * ix, 0.05 * iy, 0.0)
            for iy in range(-20, 21)
            for ix in range(-30, 31)
        ]
        return point_cloud2.create_cloud_xyz32(header, points)

    def test_rrt_star_five_centimeter_path(self, proc_info, proc_output, planner):
        proc_info.assertWaitForStartup(process=planner, timeout=10.0)
        self._spin_until(
            lambda: self.cloud_publisher.get_subscription_count() > 0
            and self.goal_publisher.get_subscription_count() > 0,
            10.0,
            'Planner ROS interfaces were not discovered',
        )
        goal = PoseStamped()
        goal.header.frame_id = MAP_FRAME
        goal.pose.position.x = 0.75
        goal.pose.orientation.w = 1.0
        self.goal_publisher.publish(goal)
        self.cloud_publisher.publish(self._flat_cloud())
        self._spin_until(
            lambda: any(path.poses for path in self.paths),
            15.0,
            'RRT* did not publish a non-empty 5 cm Path',
        )
        path = next(path for path in self.paths if path.poses)
        self.assertEqual(MAP_FRAME, path.header.frame_id)
        self.assertAlmostEqual(-0.75, path.poses[0].pose.position.x, places=6)
        self.assertAlmostEqual(0.75, path.poses[-1].pose.position.x, places=6)
        for previous, current in zip(path.poses, path.poses[1:]):
            spacing = math.hypot(
                current.pose.position.x - previous.pose.position.x,
                current.pose.position.y - previous.pose.position.y,
            )
            self.assertLessEqual(spacing, 0.05 + 1.0e-8)


@launch_testing.post_shutdown_test()
class TestRrtStarProcessExit(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
