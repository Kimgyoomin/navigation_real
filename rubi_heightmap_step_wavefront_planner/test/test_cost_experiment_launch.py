#!/usr/bin/env python3
"""Synthetic ROS topic test, not a Gazebo locomotion experiment."""
import json
import math
import tempfile
import time
import unittest
import yaml
from pathlib import Path
import launch
import launch_ros.actions
import launch_testing.actions
import pytest
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.msg import Costmap
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header, String
from std_srvs.srv import Trigger
from ament_index_python.packages import get_package_share_directory

PACKAGE = 'rubi_heightmap_step_wavefront_planner'
OUTPUT = tempfile.mkdtemp(prefix='rubi_cot_topic_test_')


@pytest.mark.launch_test
def generate_test_description():
    share = Path(get_package_share_directory(PACKAGE))
    model = json.loads((share/'config/cot_fit_240trials_20261019.json').read_text())
    common = yaml.safe_load((share/'config/grid_cost_experiment.yaml').read_text())[
        'rubi_hybrid_planner_comparison']['ros__parameters']
    common.update({
        'planner_run_mode': 'grid_only', 'replanning.enabled': False,
        'input_costmap_topic': '/test/costmap', 'input_heightmap_topic': '/test/heights',
        'comparison_goal_topic': '/test/goal', 'max_costmap_age_s': 60., 'max_heightmap_age_s': 60.,
        'node_evidence_radius_m': .025, 'node_min_observed_cells': 1,
        'node_max_nearest_evidence_distance_m': .04, 'node_max_height_outlier_ratio': 1.,
        'edge_height_query_radius_m': .04, 'edge_max_height_evidence_gap_m': .08,
        'evaluation.max_crossable_height_jump_m': .10, 'evaluation.height_cost_weight': 1.,
        'evaluation.grid_comparison_max_event_height_m': .11,
        'experiment.publish_metrics': True,
        'evaluation.grid_cot_polynomial_coefficients_cm': model['models'][2]['coefficients_descending_cm'],
        'evaluation.grid_cot_flat_baseline': model['baseline_CoT'],
    })
    nodes = [launch_ros.actions.Node(package='tf2_ros', executable='static_transform_publisher',
        arguments=['--x', '.125', '--y', '.525', '--z', '0',
                   '--frame-id', 'map', '--child-frame-id', 'base_link'])]
    for mode in ('vanilla', 'cot'):
        nodes.append(launch_ros.actions.Node(package=PACKAGE,
            executable='hybrid_planner_comparison_node', name='test_'+mode,
            parameters=[dict(common, **{'evaluation.grid_cot_height_cost_enabled': mode == 'cot',
                                      'experiment.run_id': 'test_'+mode})],
            remappings=[('/rubi/experiment/plan_metrics', '/test/'+mode+'_metrics'),
                        ('/rubi/planner_comparison/grid/path', '/test/'+mode+'_path'),
                        ('/rubi/planner_comparison/grid/tracking_path', '/test/'+mode+'_tracking')]))
    nodes.append(launch_ros.actions.Node(package=PACKAGE, executable='navigation_experiment_metrics.py',
        parameters=[{'run_id': 'test_vanilla', 'output_dir': OUTPUT,
                     'joint_topic': '/test/joints', 'odom_topic': '/test/odom'}],
        remappings=[('/rubi/experiment/plan_metrics', '/test/vanilla_metrics')]))
    return launch.LaunchDescription(nodes+[launch_testing.actions.ReadyToTest()])


class TestCostExperiment(unittest.TestCase):
    def test_topics_cost_choice_and_work(self):
        rclpy.init()
        node = rclpy.create_node('cost_experiment_test_client')
        latched = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        plans = {'vanilla': [], 'cot': []}
        subscriptions = [node.create_subscription(String, '/test/'+mode+'_metrics',
            lambda m, mode=mode: plans[mode].append(json.loads(m.data)), latched)
            for mode in plans]
        results = []
        subscriptions.append(node.create_subscription(String, '/rubi/experiment/run_metrics',
            lambda m: results.append(json.loads(m.data)), latched))
        cost_pub = node.create_publisher(Costmap, '/test/costmap', latched)
        height_pub = node.create_publisher(PointCloud2, '/test/heights', 10)
        goal_pub = node.create_publisher(PoseStamped, '/test/goal', 10)
        joint_pub = node.create_publisher(JointState, '/test/joints', 10)
        odom_pub = node.create_publisher(Odometry, '/test/odom', 10)
        t0 = node.get_clock().now().nanoseconds/1e9

        def sensors():
            stamp = node.get_clock().now().to_msg()
            j = JointState(header=Header(stamp=stamp),
                name=['L_HR_JOINT','L_HP_JOINT','L_KN_JOINT','R_HR_JOINT','R_HP_JOINT','R_KN_JOINT'],
                effort=[1.,-1.,1.,-1.,1.,-1.], velocity=[1.]*6)
            o = Odometry(header=Header(stamp=stamp, frame_id='world'), child_frame_id='base_link')
            o.pose.pose.position.x = .2*(stamp.sec+stamp.nanosec/1e9-t0)
            joint_pub.publish(j)
            odom_pub.publish(o)
        timer = node.create_timer(.01, sensors)

        def until(predicate, seconds=12.):
            deadline = time.monotonic()+seconds
            while not predicate() and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=.01)
            self.assertTrue(predicate())

        try:
            until(lambda: goal_pub.get_subscription_count() == 2 and
                  height_pub.get_subscription_count() == 2 and joint_pub.get_subscription_count() > 0)
            start = node.create_client(Trigger, '/rubi/experiment/start')
            self.assertTrue(start.wait_for_service(timeout_sec=10))
            deadline = time.monotonic()+.2
            until(lambda: time.monotonic() > deadline)
            future = start.call_async(Trigger.Request()); until(future.done)
            self.assertTrue(future.result().success, future.result().message)
            stamp = node.get_clock().now().to_msg()
            c = Costmap(header=Header(stamp=stamp, frame_id='map'))
            c.metadata.size_x, c.metadata.size_y = 41, 21
            c.metadata.resolution = .05; c.metadata.origin.orientation.w = 1.
            c.data = [0]*(41*21)
            points = [(.025+.05*x, .025+.05*y, .08 if 13<=x<=27 and 8<=y<=12 else 0.)
                      for y in range(21) for x in range(41)]
            cost_pub.publish(c)
            height_pub.publish(point_cloud2.create_cloud_xyz32(c.header, points))
            deadline = time.monotonic()+.3; until(lambda: time.monotonic() > deadline)
            goal = PoseStamped(header=Header(stamp=stamp, frame_id='map'))
            goal.pose.position.x, goal.pose.position.y, goal.pose.orientation.w = 1.925, .525, 1.
            goal_pub.publish(goal)
            until(lambda: plans['vanilla'] and plans['cot'])
            v, f = plans['vanilla'][-1], plans['cot'][-1]
            for p in (v, f):
                self.assertTrue(p['accepted'], p)
                self.assertEqual(p['parameters']['evaluation.local_relief_hard_reject_enabled'], 'true')
                self.assertAlmostEqual(float(p['parameters']['evaluation.local_relief_threshold_m']), .10)
                self.assertAlmostEqual(p['total_cost'], p['distance_weight']*p['planned_length_m']+
                    p['height_cost']+p['inflation_cost']+p['sobel_cost'], places=7)
            self.assertEqual(v['heightmap_hash'], f['heightmap_hash'])
            self.assertEqual(v['costmap_hash'], f['costmap_hash'])
            self.assertGreater(f['planned_length_m'], v['planned_length_m'])
            self.assertGreater(v['height_score_m'], f['height_score_m'])
            finish = node.create_client(Trigger, '/rubi/experiment/finish_success')
            self.assertTrue(finish.wait_for_service(timeout_sec=5))
            future = finish.call_async(Trigger.Request()); until(future.done)
            self.assertTrue(future.result().success)
            until(lambda: results)
            result = results[-1]
            self.assertTrue(result['qc_pass'], result['qc_errors'])
            m = result['metrics']
            self.assertAlmostEqual(m['W_abs_J'], 6*m['duration_s'], places=6)
            self.assertAlmostEqual(m['W_signed_J'], 0., places=6)
            self.assertAlmostEqual(m['executed_length_xy_m'], .2*m['duration_s'], places=5)
            self.assertEqual(node.count_publishers('/cmd_vel'), 0)
            Path(OUTPUT, 'topic_test_summary.json').write_text(json.dumps(
                {'synthetic_test_only': True, 'vanilla': v, 'cot': f, 'measurement': result}, indent=2))
            print('SYNTHETIC_TOPIC_TEST_RESULT='+str(Path(OUTPUT, 'topic_test_summary.json')))
        finally:
            node.destroy_timer(timer)
            node.destroy_node()
            rclpy.shutdown()
