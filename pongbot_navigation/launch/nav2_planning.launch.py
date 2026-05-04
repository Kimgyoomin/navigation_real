from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    params_arg = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(
            get_package_share_directory("pongbot_navigation"),
            "config",
            "nav2_planning.yaml",
        ),
        description="Nav2 planning-only params",
    )

    planner_server = Node(
        package="nav2_planner",
        executable="planner_server",
        name="planner_server",
        output="screen",
        parameters=[LaunchConfiguration("params_file")],
    )

    bt_navigator = Node(
        package="nav2_bt_navigator",
        executable="bt_navigator",
        name="bt_navigator",
        output="screen",
        parameters=[LaunchConfiguration("params_file")],
    )

    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_planning",
        output="screen",
        parameters=[{
            "use_sim_time": False,
            "autostart": True,
            "node_names": ["planner_server", "bt_navigator"],
        }],
    )

    # Localization/TF가 먼저 올라올 시간을 줌 (핵심!)
    delayed_nav2 = TimerAction(period=5.0, actions=[planner_server, bt_navigator, lifecycle_manager])

    return LaunchDescription([
        params_arg,
        delayed_nav2,
    ])