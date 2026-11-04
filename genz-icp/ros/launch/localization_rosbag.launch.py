from pathlib import Path

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from navigation_dds.profiles import dds_group


def bag_player(context):
    bagfile = LaunchConfiguration("bagfile").perform(context)
    if not bagfile:
        # A separately launched player must publish /clock and only input clouds.
        return []
    cmd = [
        "ros2", "bag", "play", str(Path(bagfile).expanduser()),
        "--clock", LaunchConfiguration("clock_rate"),
        "--rate", LaunchConfiguration("rate"),
        "--read-ahead-queue-size", LaunchConfiguration("read_ahead_queue_size"),
        # Do not replay old TF, localization, /initialpose or robot commands.
        "--topics", LaunchConfiguration("topic"),
    ]
    if ParameterValue(LaunchConfiguration("start_paused"), value_type=bool).evaluate(context):
        cmd.append("--start-paused")
    return [dds_group([ExecuteProcess(cmd=cmd, output="screen")], "GenZ bag player")]


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument("use_sim_time", default_value="true", choices=["true"]),
        DeclareLaunchArgument("use_sensor_stamp", default_value="true", choices=["true"]),
        DeclareLaunchArgument(
            "bagfile", default_value="",
            description="Bag directory; empty means use a separate sensor-only player.",
        ),
        DeclareLaunchArgument(
            "clock_rate", default_value="40",
            description="/clock publication Hz, independent of playback speed.",
        ),
        DeclareLaunchArgument("rate", default_value="1.0"),
        DeclareLaunchArgument("read_ahead_queue_size", default_value="100"),
        DeclareLaunchArgument(
            "start_paused", default_value="true",
            description="Set initial pose, then call /rosbag2_player/resume to start.",
        ),
    ]

    # Share topic, map, mount, frames, RViz and DDS defaults with live operation.
    # The included launch contains no player, so bagfile cannot start it twice.
    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare("genz_icp"), "launch", "localization.launch.py",
        ])),
        launch_arguments={
            "use_sim_time": "true",
            "use_sensor_stamp": "true",
        }.items(),
    )

    return LaunchDescription(declared_arguments + [
        localization,
        OpaqueFunction(function=bag_player),
    ])
