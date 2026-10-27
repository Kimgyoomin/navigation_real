"""Configuration contract; does not launch Nav2 or send motion commands."""
import ast
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest
import yaml

PACKAGE = Path(__file__).resolve().parents[1]
CONFIG = yaml.safe_load((PACKAGE / 'config/nav2_rubi_dwb.yaml').read_text())


@pytest.mark.parametrize('name, rate', [('local_costmap', 5.0), ('global_costmap', 1.0)])
def test_costmap_geometry_and_filtered_source(name, rate):
    params = CONFIG[name][name]['ros__parameters']
    assert params['update_frequency'] == rate
    assert params['publish_frequency'] == rate
    assert 'robot_radius' not in params
    footprint = ast.literal_eval(params['footprint'])
    assert len(footprint) == 4
    assert {tuple(p) for p in footprint} == {
        (.275, .275), (-.275, .275), (-.275, -.275), (.275, -.275)}
    assert params['footprint_padding'] == .01
    source = params['obstacle_layer']['livox_cloud']
    assert source['topic'] == '/livox/lidar_costmap'
    assert source['obstacle_min_range'] == .3
    assert source['data_type'] == 'PointCloud2'
    assert source['marking'] and source['clearing']
    assert params['plugins'][-1] == 'inflation_layer'


def test_filter_contract_and_raw_input_is_untouched():
    params = CONFIG['costmap_sector_filter']['ros__parameters']
    assert params['input_topic'] == '/livox/lidar_pointcloud'
    assert params['output_topic'] == '/livox/lidar_costmap'
    assert params['expected_frame_id'] == 'livox_frame'
    assert params['front_half_angle_deg'] == 60.
    assert params['front_min_range'] == .3
    assert params['other_min_range'] == 1.
    for package in ('FAST_LIO_ROS2', 'FAST_LIO_LOCALIZATION2'):
        config = yaml.safe_load((PACKAGE.parent / package / 'config/mid360.yaml').read_text())
        assert config['/**']['ros__parameters']['common']['lid_topic'] == '/livox/lidar'


def test_controller_keeps_baseline_velocity_and_checks_corners():
    controller = CONFIG['controller_server']['ros__parameters']
    assert controller['odom_topic'] == '/Odometry'
    assert controller['controller_frequency'] == 5.
    assert controller['FollowPath']['closed_loop'] is True
    assert 'BaseObstacle' in controller['FollowPath']['critics']
    assert 'ObstacleFootprint' in controller['FollowPath']['critics']
    assert controller['FollowPath']['ObstacleFootprint.scale'] > 0.
    assert controller['FollowPath']['primary_controller'] == 'dwb_core::DWBLocalPlanner'


def test_bt_replan_rate_and_warning_budget():
    tree = ET.parse(PACKAGE / 'behavior_trees/navigate_to_pose_rubi.xml')
    assert float(tree.find('.//RateController').attrib['hz']) == 1.
    assert CONFIG['planner_server']['ros__parameters']['expected_planner_frequency'] == 1.


def test_launch_includes_filter_with_parameter_file():
    tree = ast.parse((PACKAGE / 'launch/rubi_navigation_dwb.launch.py').read_text())
    nodes = [node for node in ast.walk(tree) if isinstance(node, ast.Call)
             and isinstance(node.func, ast.Name) and node.func.id == 'Node']
    filters = [node for node in nodes if any(k.arg == 'executable'
               and isinstance(k.value, ast.Constant) and k.value.value == 'costmap_sector_filter'
               for k in node.keywords)]
    assert len(filters) == 1
    assert any(k.arg == 'parameters' for k in filters[0].keywords)


def test_current_mount_axes_match_the_sensor_frame_filter_assumption():
    launch = PACKAGE.parent / 'FAST_LIO_LOCALIZATION2/launch/localization_nav_rubi.launch.py'
    tree = ast.parse(launch.read_text())
    transforms = {}
    for node in ast.walk(tree):
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Name):
            if node.func.id == 'static_transform_node':
                args = {k.arg: ast.literal_eval(k.value) for k in node.keywords}
                transforms[args['name']] = args
    for name in ('body_to_livox_frame', 'body_to_robot_body', 'robot_body_to_base_link'):
        for axis in ('roll', 'pitch', 'yaw'):
            assert float(transforms[name].get(axis, '0.0')) == 0.
