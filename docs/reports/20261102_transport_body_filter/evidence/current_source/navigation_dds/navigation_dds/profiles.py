"""Explicit SHM + UDP profiles, without changing endpoint QoS or timestamps."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, GroupAction, LogInfo
from launch.actions import OpaqueFunction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration


PROFILE_ENV_KEYS = ('FASTRTPS_DEFAULT_PROFILES_FILE', 'FASTDDS_DEFAULT_PROFILES_FILE')
PROFILE_MIB = {'common': 32, 'large': 128}
UDP_BUFFER_MIB = 4


def profile_environment(profile, environment):
    """Return only the overrides; retain the caller's other environment values."""
    if profile not in PROFILE_MIB:
        raise ValueError('Unknown DDS profile: ' + profile)
    # Humble rmw_fastrtps adds another default-sized SHM transport in this mode.
    if environment.get('ROS_LOCALHOST_ONLY') == '1':
        raise RuntimeError(
            'RUBI DDS profiles require ROS_LOCALHOST_ONLY=0 or unset. '
            'Use dds_profile:=default to retain your inherited DDS configuration.')
    rmw = environment.get('RMW_IMPLEMENTATION', '')
    if rmw and rmw not in ('rmw_fastrtps_cpp', 'rmw_fastrtps_dynamic_cpp'):
        raise RuntimeError(
            'RUBI SHM profiles require Fast DDS; RMW_IMPLEMENTATION=' + rmw +
            '. Use dds_profile:=default for another middleware.')
    path = Path(get_package_share_directory('navigation_dds')) / 'config' / (
        'fastdds_' + profile + '.xml')
    if not path.is_file():
        raise RuntimeError('DDS profile is missing: ' + str(path))
    return {key: str(path) for key in PROFILE_ENV_KEYS}


def dds_argument():
    return DeclareLaunchArgument(
        'dds_profile', default_value='navigation', choices=['navigation', 'default'],
        description='navigation: 32 MiB SHM, PCD sender 128 MiB; '
                    'DDS UDP send/receive buffer requests 4 MiB each; '
                    'default: retain inherited DDS settings')


def _configure(context, label, profile):
    mode = LaunchConfiguration('dds_profile').perform(context)
    if mode == 'default':
        return [LogInfo(msg=label + ' DDS profile: inherited settings')]
    if mode != 'navigation':
        raise RuntimeError('Unknown dds_profile: ' + mode)
    environment = profile_environment(profile, context.environment)
    return [
        *[SetEnvironmentVariable(key, value) for key, value in environment.items()],
        LogInfo(msg=f'{label} DDS profile: {PROFILE_MIB[profile]} MiB SHM + UDP '
                    f'(per DDS participant); DDS UDP buffer requests: '
                    f'TX/RX {UDP_BUFFER_MIB} MiB each (kernel caps apply)'),
    ]


def dds_group(actions, label, profile='common'):
    """Scope XML overrides to these children; a nested PCD group can use 128 MiB."""
    return GroupAction(scoped=True, actions=[
        OpaqueFunction(function=_configure, args=[label, profile]),
        *actions,
    ])
