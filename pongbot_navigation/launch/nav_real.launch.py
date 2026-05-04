from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition

import os


def generate_launch_description():
    # -------------------------
    # Launch args (paths)
    # -------------------------
    pcd_map_path_arg = DeclareLaunchArgument(
        "pcd_map_path",
        default_value="/home/kim/ros2_ws/src/FAST_LIO_LOCALIZATION2/PCD/scans_down.pcd",
        description="PCD map file path for global localization"
    )

    pcd_map_topic_arg = DeclareLaunchArgument(
        "pcd_map_topic",
        default_value="/cloud_pcd",
        description="PointCloud2 topic name for PCD debug"
    )

    map_yaml_arg = DeclareLaunchArgument(
        "map_yaml",
        default_value="/home/kim/ros2_ws/src/FAST_LIO_LOCALIZATION2/maps/building_1f_map.yaml",
        description="2D occupancy map yaml file path"
    )

    publish_2d_map_arg = DeclareLaunchArgument(
        "publish_2d_map",
        default_value="true",
        description="Whether to publish /map via nav2_map_server inside localization module"
    )

    use_rviz_arg = DeclareLaunchArgument(
        "rviz",
        default_value="false",
        description="Start RViz from this launch (recommend false; run RViz separately or from Nav2)"
    )

    nav2_params_arg = DeclareLaunchArgument(
        "nav2_params",
        default_value=os.path.join(
            get_package_share_directory("pongbot_navigation"),
            "config",
            "nav2_params.yaml",
        ),
        description="Nav2 parameters yaml"
    )

    autostart_arg = DeclareLaunchArgument(
        "autostart",
        default_value="true",
        description="Autostart lifecycle nodes"
    )

    # -------------------------
    # Include: fast_lio_localization localization.launch.py
    # -------------------------
    fastlio_pkg = get_package_share_directory("fast_lio_localization")
    fastlio_launch = os.path.join(fastlio_pkg, "launch", "localization.launch.py")

    localization_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(fastlio_launch),
        launch_arguments={
            "map": LaunchConfiguration("pcd_map_path"),
            "pcd_map_topic": LaunchConfiguration("pcd_map_topic"),
            "map_yaml": LaunchConfiguration("map_yaml"),
            "publish_2d_map": LaunchConfiguration("publish_2d_map"),
            "rviz": "false",   # RViz는 여기서 끄는 걸 권장(중복 방지)
        }.items(),
    )

    # -------------------------
    # Nav2 nodes (minimal end-to-end for planning)
    # - planner_server (A* plugin)
    # - bt_navigator (goal receiver)
    # - lifecycle_manager (activate)
    # Optional: controller_server, behavior_server, waypoint_follower
    # -------------------------
    nav2_params = LaunchConfiguration("nav2_params")

    planner_server = Node(
        package="nav2_planner",
        executable="planner_server",
        name="planner_server",
        output="screen",
        parameters=[nav2_params],
    )

    bt_navigator = Node(
        package="nav2_bt_navigator",
        executable="bt_navigator",
        name="bt_navigator",
        output="screen",
        parameters=[nav2_params],
    )

    # Optional (enable later when planning-only passes):
    controller_server = Node(
        package="nav2_controller",
        executable="controller_server",
        name="controller_server",
        output="screen",
        parameters=[nav2_params],
    )

    behavior_server = Node(
        package="nav2_behaviors",
        executable="behavior_server",
        name="behavior_server",
        output="screen",
        parameters=[nav2_params],
    )

    # If I want to use map_2d
    map_to_map2d_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="map_to_map2d",
        arguments=["0", "0", "0", "0", "0", "0", "map", "map_2d"],
        output="screen",
    )


    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
        output="screen",
        parameters=[{
            "use_sim_time": False,
            "autostart": LaunchConfiguration("autostart"),
            "node_names": [
                "planner_server",
                # "controller_server",
                "bt_navigator",
                # "behavior_server",
            ],
        }],
    )

    # RViz (optional)
    # rviz_node = Node(
    #     package="rviz2",
    #     executable="rviz2",
    #     name="rviz2",
    #     output="screen",
    #     condition=None,
    #     arguments=[],
    # )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        condition=IfCondition(LaunchConfiguration("rviz")),
        arguments=["-d", "/home/kim/ros2_ws/src/FAST_LIO_LOCALIZATION2/rviz/fastlio_localization.rviz"],
    )

    # NOTE:
    # RViz 조건부 실행을 제대로 하려면 IfCondition 사용이 필요하지만,
    # 지금은 혼선 줄이려고 RViz는 기본 false로 두고 외부 실행 권장.

    return LaunchDescription([
        pcd_map_path_arg,
        pcd_map_topic_arg,
        map_yaml_arg,
        publish_2d_map_arg,
        use_rviz_arg,
        nav2_params_arg,
        autostart_arg,

        localization_include,

        # Nav2 core
        planner_server,
        # controller_server,
        bt_navigator,
        # behavior_server,
        lifecycle_manager,
        rviz_node,
    ])