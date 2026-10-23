"""MID-360S alias retaining navigation_real's CustomMsg + PointCloud2 output."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    path = Path(get_package_share_directory('livox_ros_driver2')) / 'launch_ROS2/rviz_MID360_launch.py'
    return LaunchDescription([IncludeLaunchDescription(PythonLaunchDescriptionSource(str(path)))])
