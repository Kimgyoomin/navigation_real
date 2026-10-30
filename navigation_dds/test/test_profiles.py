"""Check actual launch scoping without starting any node or sending any command."""
import importlib.util
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest
from launch import LaunchContext
from launch.actions import ExecuteProcess, OpaqueFunction
from launch.utilities import perform_substitutions, visit_all_entities_and_collect_futures
from launch.utilities import normalize_to_list_of_substitutions
from launch_ros.actions import Node

from navigation_dds.profiles import dds_group, profile_environment, PROFILE_ENV_KEYS


ROOT = Path(__file__).resolve().parents[2]
NS = {'f': 'http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles'}


def test_profile_transport_sizes_and_no_endpoint_qos_changes():
    for filename, mib in [('fastdds_common.xml', 32), ('fastdds_large.xml', 128)]:
        tree = ET.parse(ROOT / 'navigation_dds/config' / filename)
        transports = tree.findall('.//f:transport_descriptor', NS)
        assert [t.find('f:type', NS).text for t in transports] == ['SHM', 'UDPv4']
        assert int(transports[0].find('f:segment_size', NS).text) == mib * 1024**2
        assert tree.find('.//f:useBuiltinTransports', NS).text == 'false'
        assert tree.find('f:participant', NS).get('is_default_profile') == 'true'
        assert tree.find('.//f:data_writer', NS) is None
        assert tree.find('.//f:data_reader', NS) is None
    driver = ET.parse(ROOT / 'livox_ros_driver2/config/fastdds_livox_shm.xml')
    assert int(driver.find('.//f:segment_size', NS).text) == 32 * 1024**2


def test_incompatible_environment_fails_explicitly():
    with pytest.raises(RuntimeError, match='ROS_LOCALHOST_ONLY'):
        profile_environment('common', {'ROS_LOCALHOST_ONLY': '1'})
    with pytest.raises(RuntimeError, match='Fast DDS'):
        profile_environment('common', {'RMW_IMPLEMENTATION': 'rmw_cyclonedds_cpp'})


def test_nested_large_profile_does_not_leak_to_later_nodes_or_parent():
    context = LaunchContext()
    context.launch_configurations['dds_profile'] = 'navigation'
    context.environment['ROS_LOCALHOST_ONLY'] = '0'
    context.environment['RMW_IMPLEMENTATION'] = 'rmw_fastrtps_cpp'
    for key in PROFILE_ENV_KEYS:
        context.environment[key] = '/original.xml'
    observed = []

    def capture(ctx):
        observed.append(ctx.environment[PROFILE_ENV_KEYS[0]])

    visit_all_entities_and_collect_futures(dds_group([
        OpaqueFunction(function=capture),
        dds_group([OpaqueFunction(function=capture)], 'PCD', 'large'),
        OpaqueFunction(function=capture),
    ], 'Common'), context)
    assert [Path(p).name for p in observed] == [
        'fastdds_common.xml', 'fastdds_large.xml', 'fastdds_common.xml']
    assert all(context.environment[key] == '/original.xml' for key in PROFILE_ENV_KEYS)


@pytest.mark.parametrize('relative,expected_count,has_pcd', [
    ('FAST_LIO_LOCALIZATION2/launch/localization_nav_rubi.launch.py', 9, True),
    ('pongbot_navigation/launch/rubi_navigation_dwb.launch.py', 9, False),
    ('pongbot_lcm/launch/joy_lcm.launch.py', 1, False),
])
@pytest.mark.parametrize('mode', ['navigation', 'default'])
def test_actual_launches_choose_correct_profiles_without_running_nodes(
        monkeypatch, relative, expected_count, has_pcd, mode):
    context = LaunchContext()
    context.launch_configurations['dds_profile'] = mode
    context.environment['ROS_LOCALHOST_ONLY'] = '0'
    context.environment['RMW_IMPLEMENTATION'] = 'rmw_fastrtps_cpp'
    for key in PROFILE_ENV_KEYS:
        context.environment[key] = '/inherited.xml'
    seen = []

    def capture(node, ctx):
        seen.append((perform_substitutions(
                         ctx, normalize_to_list_of_substitutions(node.node_executable)),
                     {k: ctx.environment.get(k) for k in PROFILE_ENV_KEYS}))

    def forbidden(*args, **kwargs):
        raise AssertionError('Test attempted to run a real process')

    monkeypatch.setattr(ExecuteProcess, 'execute', forbidden)
    monkeypatch.setattr(Node, 'execute', capture)
    spec = importlib.util.spec_from_file_location('launch_under_test', ROOT / relative)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    visit_all_entities_and_collect_futures(module.generate_launch_description(), context)
    assert len(seen) == expected_count
    assert sum(executable == 'pcd_to_pointcloud' for executable, _ in seen) == int(has_pcd)
    for executable, environment in seen:
        expected = ('inherited.xml' if mode == 'default' else
                    'fastdds_large.xml' if executable == 'pcd_to_pointcloud' else
                    'fastdds_common.xml')
        assert all(Path(value).name == expected for value in environment.values())
    assert all(context.environment[key] == '/inherited.xml' for key in PROFILE_ENV_KEYS)
