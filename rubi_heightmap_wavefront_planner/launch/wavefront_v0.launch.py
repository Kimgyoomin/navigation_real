from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(
        get_package_share_directory("rubi_heightmap_wavefront_planner")
    )
    default_config = str(package_share / "config" / "wavefront_v0.yaml")
    default_rviz = str(package_share / "rviz" / "wavefront_v0.rviz")

    config = LaunchConfiguration("config")
    rviz_config = LaunchConfiguration("rviz_config")
    use_sim_time = LaunchConfiguration("use_sim_time")
    launch_rviz = LaunchConfiguration("launch_rviz")
    base_frame = LaunchConfiguration("base_frame")

    return LaunchDescription(
        [
            DeclareLaunchArgument("config", default_value=default_config),
            DeclareLaunchArgument("rviz_config", default_value=default_rviz),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("launch_rviz", default_value="true"),
            DeclareLaunchArgument("base_frame", default_value="body"),
            Node(
                package="rubi_heightmap_wavefront_planner",
                executable="wavefront_planner_node",
                name="rubi_heightmap_wavefront_planner",
                output="screen",
                parameters=[
                    config,
                    {"use_sim_time": use_sim_time, "base_frame": base_frame},
                ],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="wavefront_rviz",
                output="screen",
                arguments=["-d", rviz_config],
                parameters=[{"use_sim_time": use_sim_time}],
                condition=IfCondition(launch_rviz),
            ),
        ]
    )
