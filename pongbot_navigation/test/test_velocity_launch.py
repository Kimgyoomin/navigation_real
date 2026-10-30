"""Resolve launch parameters without starting any ROS process or publishing commands."""
import importlib.util
from pathlib import Path

import pytest
import yaml
from launch import LaunchContext
from launch.actions import OpaqueFunction
from launch_ros.utilities import evaluate_parameters

PACKAGE = Path(__file__).resolve().parents[1]


def consumers(flag, params_path):
    spec = importlib.util.spec_from_file_location('velocity_launch', PACKAGE / 'launch/rubi_navigation_dwb.launch.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    context = LaunchContext()
    context.launch_configurations.update({
        'use_fastlio_velocity': flag, 'params_file': str(params_path), 'use_sim_time': 'false'})

    def walk(action):
        # ROS Humble's OpaqueFunction exposes its callable only privately.
        if isinstance(action, OpaqueFunction) and action._OpaqueFunction__function.__name__ == 'nav2_consumers':
            yield from action.execute(context)
        for child in action.get_sub_entities() or []:
            yield from walk(child)
    nodes = [node for action in module.generate_launch_description().entities for node in walk(action)]
    return [evaluate_parameters(context, node._Node__parameters) for node in nodes]


def test_disabled_feedback_keeps_yaml_without_overrides():
    params = consumers('false', PACKAGE / 'config/nav2_rubi_dwb.yaml')
    assert len(params) == 2
    assert all(p[-1] == {} for p in params)


@pytest.mark.parametrize('hz', [5., 10.])
def test_enabled_feedback_matches_control_period_and_both_consumers(hz, tmp_path):
    config = yaml.safe_load((PACKAGE / 'config/nav2_rubi_dwb.yaml').read_text())
    config['controller_server']['ros__parameters']['controller_frequency'] = hz
    path = tmp_path / 'nav2.yaml'
    path.write_text(yaml.safe_dump(config))
    controller, bt = consumers('true', path)
    assert controller[-1]['odom_topic'] == '/fastlio/odometry_base'
    assert bt[-1]['odom_topic'] == '/fastlio/odometry_base'
    assert controller[-1]['FollowPath.trajectory_generator_name'] == 'dwb_plugins::LimitedAccelGenerator'
    assert controller[-1]['FollowPath.sim_period'] == pytest.approx(1./hz)


def test_invalid_controller_rate_rejected(tmp_path):
    config = yaml.safe_load((PACKAGE / 'config/nav2_rubi_dwb.yaml').read_text())
    config['controller_server']['ros__parameters']['controller_frequency'] = 0.
    path = tmp_path / 'nav2.yaml'; path.write_text(yaml.safe_dump(config))
    with pytest.raises(ValueError, match='controller_frequency'):
        consumers('true', path)
