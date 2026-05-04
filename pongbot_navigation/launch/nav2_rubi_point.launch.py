#!/usr/bin/env python3
"""
nav2_rubi_point.launch.py

FAST-LIO-LOCALIZATION + Nav2 launch file for Rubi/PongBot.

Assumed TF semantics:
  FAST-LIO body      : FRD  (x front, y right, z down), approximately Livox origin
  Nav2 base_link     : FLU  (x front, y left,  z up), robot base / footprint frame
  Livox livox_frame  : FRD  (same orientation/origin as FAST-LIO body, unless calibrated otherwise)

Default physical mounting assumption:
  base_link -> livox_frame translation in base_link(FLU):
    x = +0.20 m  # Livox is 20 cm in front of base_link
    y = +0.00 m
    z = +0.20 m  # Livox is 20 cm above base_link

With FRD <-> FLU conversion using Rx(pi):
  body -> base_link translation becomes [-x, +y, +z]
  body -> base_link rotation is roll = pi
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml
from ament_index_python.packages import get_package_share_directory
import os


PI = "3.141592653589793"
NAV2_START_DELAY_SEC = 30.0


def static_transform_node(
    *,
    name,
    frame_id,
    child_frame_id,
    x="0.0",
    y="0.0",
    z="0.0",
    roll="0.0",
    pitch="0.0",
    yaw="0.0",
    condition=None,
):
    """Create a Humble-compatible tf2_ros static_transform_publisher node."""
    return Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name=name,
        output="screen",
        condition=condition,
        arguments=[
            "--x", x,
            "--y", y,
            "--z", z,
            "--roll", roll,
            "--pitch", pitch,
            "--yaw", yaw,
            "--frame-id", frame_id,
            "--child-frame-id", child_frame_id,
        ],
    )


def generate_launch_description():
    # -------------------------------------------------------------------------
    # Launch arguments
    # -------------------------------------------------------------------------
    params_file = LaunchConfiguration("params_file")
    rviz_config = LaunchConfiguration("rviz_config")
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_rviz = LaunchConfiguration("use_rviz")

    pcd_map = LaunchConfiguration("pcd_map")
    pcd_map_topic = LaunchConfiguration("pcd_map_topic")
    map_yaml = LaunchConfiguration("map_yaml")
    publish_2d_map = LaunchConfiguration("publish_2d_map")

    publish_livox_tf = LaunchConfiguration("publish_livox_tf")
    base_to_livox_x = LaunchConfiguration("base_to_livox_x")
    base_to_livox_y = LaunchConfiguration("base_to_livox_y")
    base_to_livox_z = LaunchConfiguration("base_to_livox_z")

    declared_arguments = [
        DeclareLaunchArgument(
            "params_file",
            default_value="/home/ams4976/ros2_ws/src/pongbot_navigation/config/nav2_rubi_pointb.yaml",
            description="Nav2 parameter YAML. Change this to nav2_rubi_point.yaml only after creating that file.",
        ),
        DeclareLaunchArgument(
            "rviz_config",
            default_value="/home/ams4976/ros2_ws/src/FAST_LIO_LOCALIZATION2/rviz/fastlio_localization.rviz",
            description="RViz config file.",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation time. For the real robot, keep false.",
        ),
        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
            description="Whether to launch RViz2 from this launch file.",
        ),
        DeclareLaunchArgument(
            "pcd_map",
            default_value="/home/ams4976/ros2_ws/src/FAST_LIO_LOCALIZATION2/PCD/test.pcd",
            description="PCD map used by FAST-LIO-LOCALIZATION.",
        ),
        DeclareLaunchArgument(
            "pcd_map_topic",
            default_value="/cloud_pcd",
            description="Published PCD map topic from localization launch.",
        ),
        DeclareLaunchArgument(
            "map_yaml",
            default_value="/home/ams4976/ros2_ws/src/pongbot_navigation/maps/building_1f_map_nav2_yaw0.yaml",
            description="2D occupancy map YAML used for Nav2/global costmap.",
        ),
        DeclareLaunchArgument(
            "publish_2d_map",
            default_value="true",
            description="Ask FAST-LIO-LOCALIZATION launch to publish the 2D map.",
        ),
        DeclareLaunchArgument(
            "publish_livox_tf",
            default_value="true",
            description=(
                "Publish base_link -> livox_frame here. Set false if URDF, robot_state_publisher, "
                "Livox driver, or another launch file already publishes livox_frame."
            ),
        ),
        DeclareLaunchArgument(
            "base_to_livox_x",
            default_value="0.20",
            description="base_link(FLU) -> livox_frame(FRD) x [m]. Positive means Livox is in front of base_link.",
        ),
        DeclareLaunchArgument(
            "base_to_livox_y",
            default_value="0.0",
            description="base_link(FLU) -> livox_frame(FRD) y [m]. Positive means Livox is left of base_link.",
        ),
        DeclareLaunchArgument(
            "base_to_livox_z",
            default_value="0.20",
            description="base_link(FLU) -> livox_frame(FRD) z [m]. Positive means Livox is above base_link.",
        ),
    ]

    # -------------------------------------------------------------------------
    # Nav2 parameters
    # -------------------------------------------------------------------------
    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key="",
            param_rewrites={"use_sim_time": use_sim_time},
            convert_types=True,
        ),
        allow_substs=True,
    )

    # -------------------------------------------------------------------------
    # FAST-LIO-LOCALIZATION include
    # -------------------------------------------------------------------------
    fastlio_launch = os.path.join(
        get_package_share_directory("fast_lio_localization"),
        "launch",
        "localization.launch.py",
    )

    # -------------------------------------------------------------------------
    # GenZ-ICP Localization include
    # -------------------------------------------------------------------------
    # fastlio_launch = os.path.join(
    #     get_package_share_directory("genz_icp"),
    #     "launch",
    #     "localization.launch.py",
    # )

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(fastlio_launch),
        launch_arguments={
            "map": pcd_map,
            "pcd_map_topic": pcd_map_topic,
            "map_yaml": map_yaml,
            "publish_2d_map": publish_2d_map,
            "rviz": "false",
        }.items(),
    )

    # -------------------------------------------------------------------------
    # TF: FAST-LIO body(FRD) -> Nav2 base_link(FLU)
    #
    # If body == livox_frame and base_link -> livox_frame is:
    #   [x, y, z] in base_link(FLU), rotation Rx(pi),
    # then the inverse body -> base_link is:
    #   [-x, y, z] in body(FRD), rotation Rx(pi).
    # -------------------------------------------------------------------------
    body_to_base_link = static_transform_node(
        name="body_to_base_link",
        frame_id="body",
        child_frame_id="base_link",
        x=PythonExpression(["-(", base_to_livox_x, ")"]),
        y=base_to_livox_y,
        z=base_to_livox_z,
        roll=PI,
        pitch="0.0",
        yaw="0.0",
    )

    # -------------------------------------------------------------------------
    # TF: Nav2 base_link(FLU) -> Livox livox_frame(FRD)
    #
    # This should be published exactly once. Disable publish_livox_tf if a URDF
    # or another launch file already publishes base_link -> livox_frame.
    # -------------------------------------------------------------------------
    base_link_to_livox_frame = static_transform_node(
        name="base_link_to_livox_frame",
        frame_id="base_link",
        child_frame_id="livox_frame",
        x=base_to_livox_x,
        y=base_to_livox_y,
        z=base_to_livox_z,
        roll=PI,
        pitch="0.0",
        yaw="0.0",
        condition=IfCondition(publish_livox_tf),
    )

    # -------------------------------------------------------------------------
    # RViz
    # -------------------------------------------------------------------------
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        condition=IfCondition(use_rviz),
    )

    # -------------------------------------------------------------------------
    # Nav2 servers
    # -------------------------------------------------------------------------
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
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "autostart": True,
                "node_names": [
                    "controller_server",
                    "smoother_server",
                    "planner_server",
                    "behavior_server",
                    "bt_navigator",
                ],
            }
        ],
    )

    # Give localization, map publication, and TF_static a chance to appear before
    # Nav2 lifecycle activation. Tune this constant if needed.
    delayed_nav2 = TimerAction(
        period=NAV2_START_DELAY_SEC,
        actions=[
            controller_server,
            smoother_server,
            planner_server,
            behavior_server,
            bt_navigator,
            lifecycle_manager,
        ],
    )

    return LaunchDescription(
        declared_arguments
        + [
            localization,
            body_to_base_link,
            base_link_to_livox_frame,
            rviz,
            delayed_nav2,
        ]
    )
