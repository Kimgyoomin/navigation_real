from nav2_common.launch import RewrittenYaml
from launch_ros.descriptions import ParameterFile

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    params_arg = DeclareLaunchArgument(
        "params_file",
        default_value="/home/ams4976/ros2_ws/src/pongbot_navigation/config/nav2_pointb.yaml",
        description="Nav2 PointB params",
    )

    rviz_arg = DeclareLaunchArgument(
        "rviz_config",
        default_value="/home/ams4976/ros2_ws/src/FAST_LIO_LOCALIZATION2/rviz/fastlio_localization.rviz",
        description="RViz config to use",
    )

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=LaunchConfiguration("params_file"),
            root_key="",
            param_rewrites={},
            convert_types=True,
        ),
        allow_substs=True,
    )

    fastlio_launch = os.path.join(
        get_package_share_directory("fast_lio_localization"),
        "launch",
        "localization.launch.py",
    )

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(fastlio_launch),
        launch_arguments={
            "map": "/home/ams4976/ros2_ws/src/FAST_LIO_LOCALIZATION2/PCD/test.pcd",
            "pcd_map_topic": "/cloud_pcd",
            "map_yaml": "/home/ams4976/ros2_ws/src/pongbot_navigation/maps/building_1f_map_flip_y.yaml",
            "publish_2d_map": "true",
            "rviz": "false",
        }.items(),
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", LaunchConfiguration("rviz_config")],
    )

    livox_tf = Node(
    package="tf2_ros",
    executable="static_transform_publisher",
    name="base_link_to_livox",
    output="screen",
    arguments=[
        "--x", "0",
        "--y", "0",
        "--z", "0",
        "--roll", "3.141592653589793",
        "--pitch", "0",
        "--yaw", "0",
        "--frame-id", "base_link",
        "--child-frame-id", "livox_frame",
    ],
)

    base_alias_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="body_to_base_link",
        output="screen",
        arguments=["0", "0", "0", "0", "0", "0", "body", "base_link"],
    )

    planner_server = Node(
        package="nav2_planner",
        executable="planner_server",
        name="planner_server",
        output="screen",
        parameters=[configured_params],
    )

    controller_server = Node(
        package="nav2_controller",
        executable="controller_server",
        name="controller_server",
        output="screen",
        parameters=[configured_params],
    )

    smoother_server = Node(
        package="nav2_smoother",
        executable="smoother_server",
        name="smoother_server",
        output="screen",
        parameters=[configured_params],
    )

    behavior_server = Node(
        package="nav2_behaviors",
        executable="behavior_server",
        name="behavior_server",
        output="screen",
        parameters=[configured_params],
    )

    bt_navigator = Node(
        package="nav2_bt_navigator",
        executable="bt_navigator",
        name="bt_navigator",
        output="screen",
        parameters=[configured_params],
    )

    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
        output="screen",
        parameters=[{
            "use_sim_time": False,
            "autostart": True,
            "node_names": [
                "planner_server",
                "controller_server",
                "smoother_server",
                "behavior_server",
                "bt_navigator",
                # "global_costmap/global_costmap",
                # "local_costmap/local_costmap",
            ],
        }],
    )

    delayed_nav2 = TimerAction(
        period=30.0,
        actions=[
            planner_server,
            controller_server,
            smoother_server,
            behavior_server,
            bt_navigator,
            lifecycle_manager,
        ],
    )

    return LaunchDescription([
        params_arg,
        rviz_arg,
        localization,
        livox_tf,
        base_alias_tf,
        rviz,
        delayed_nav2,
    ])