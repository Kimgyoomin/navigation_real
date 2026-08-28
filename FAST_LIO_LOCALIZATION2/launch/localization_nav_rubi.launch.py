#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


PI = "3.141592653589793"


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
    pcd_map = LaunchConfiguration("pcd_map")
    pcd_map_topic = LaunchConfiguration("pcd_map_topic")
    map_yaml = LaunchConfiguration("map_yaml")
    publish_2d_map = LaunchConfiguration("publish_2d_map")

    use_sim_time = LaunchConfiguration("use_sim_time")

    use_rviz = LaunchConfiguration("use_rviz")
    rviz_config = LaunchConfiguration("rviz_config")

    # publish_livox_tf = LaunchConfiguration("publish_livox_tf")
    # base_to_livox_x = LaunchConfiguration("base_to_livox_x")
    # base_to_livox_y = LaunchConfiguration("base_to_livox_y")
    # base_to_livox_z = LaunchConfiguration("base_to_livox_z")

    declared_arguments = [
        DeclareLaunchArgument(
            "pcd_map",
            default_value="/home/rclab/ros2_ws/src/navigation_real/FAST_LIO_LOCALIZATION2/PCD/1f_fast_lio_260731_segmented.pcd",
        ),
        DeclareLaunchArgument(
            "pcd_map_topic",
            default_value="/cloud_pcd",
        ),
        DeclareLaunchArgument(
            "map_yaml",
            default_value="false",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation (Gazebo) clock if true",
        ),

        # Split architecture에서는 navigation.launch.py의 map_server가 /map을 담당한다.
        # FAST-LIO 쪽에서 /map까지 publish하면 중복될 수 있으므로 기본 false 추천.
        DeclareLaunchArgument("publish_2d_map", default_value="false"),

        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument(
            "rviz_config",
            default_value="/home/rclab/ros2_ws/src/navigation_real/FAST_LIO_LOCALIZATION2/rviz/fastlio_localization.rviz",
        ),

        # DeclareLaunchArgument("publish_livox_tf", default_value="true"),
        # DeclareLaunchArgument("base_to_livox_x", default_value="0.20"),
        # DeclareLaunchArgument("base_to_livox_y", default_value="0.0"),
        # DeclareLaunchArgument("base_to_livox_z", default_value="0.20"),
    ]

    fastlio_launch = os.path.join(
        get_package_share_directory("fast_lio_localization"),
        "launch",
        "localization.launch.py",
    )

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(fastlio_launch),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "map": pcd_map,
            "pcd_map_topic": pcd_map_topic,
            "map_yaml": map_yaml,
            "publish_2d_map": publish_2d_map,
            "rviz": "false",
        }.items(),
    )

    # body_to_base_link = static_transform_node(
    #     name="body_to_base_link",
    #     frame_id="body",
    #     child_frame_id="base_link",
    #     x=PythonExpression(["-(", base_to_livox_x, ")"]),
    #     y=base_to_livox_y,
    #     z=base_to_livox_z,
    #     roll=PI,
    #     pitch="0.0",
    #     yaw="0.0",
    # )

    # base_link_to_livox_frame = static_transform_node(
    #     name="base_link_to_livox_frame",
    #     frame_id="base_link",
    #     child_frame_id="livox_frame",
    #     x=base_to_livox_x,
    #     y=base_to_livox_y,
    #     z=base_to_livox_z,
    #     roll=PI,
    #     pitch="0.0",
    #     yaw="0.0",
    #     condition=IfCondition(publish_livox_tf),
    # )
    body_to_robot_body = static_transform_node(
        name="body_to_robot_body",
        frame_id="body",
        child_frame_id="BODY",
        x="-0.112",
        y="0.02329",
        z="0.07278",
        roll="0.0",
        pitch="0.0",
        yaw="0.0",
    )

    body_to_livox_frame = static_transform_node(
        name="body_to_livox_frame",
        frame_id="body",
        child_frame_id="livox_frame",
        x="-0.011",
        y="0.02329",
        z="-0.04412",
        roll="0.0",
        pitch="0.0",
        yaw="0.0",
    )

    robot_body_to_base_link = static_transform_node(
        name="robot_body_to_base_link",
        frame_id="BODY",
        child_frame_id="base_link",
        x="0.0",
        y="0.0",
        z="0.0",
        roll="0.0",
        pitch="0.0",
        yaw="0.0",
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_fastlio_localization",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[{
            "use_sim_time": use_sim_time,
        }],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription(
        declared_arguments
        + [
            localization,
            body_to_livox_frame,
            body_to_robot_body,
            robot_body_to_base_link,
            rviz,
        ]
    )