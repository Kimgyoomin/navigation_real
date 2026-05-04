#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


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
    params_file = LaunchConfiguration("params_file")
    map_yaml = LaunchConfiguration("map_yaml")
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")

    publish_livox_tf = LaunchConfiguration("publish_livox_tf")
    base_to_livox_x = LaunchConfiguration("base_to_livox_x")
    base_to_livox_y = LaunchConfiguration("base_to_livox_y")
    base_to_livox_z = LaunchConfiguration("base_to_livox_z")

    declared_arguments = [
        DeclareLaunchArgument(
            "params_file",
            default_value="/home/ams4976/ros2_ws/src/pongbot_navigation/config/nav2_pointb.yaml",
            description="Nav2 PointB params",
        ),
        DeclareLaunchArgument(
            "map_yaml",
            default_value="/home/ams4976/ros2_ws/src/pongbot_navigation/maps/building_1f_map_nav2_yaw0.yaml",
            description="2D occupancy map used by Nav2 static layer",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation time",
        ),
        DeclareLaunchArgument(
            "autostart",
            default_value="true",
            description="Automatically activate lifecycle nodes",
        ),
        DeclareLaunchArgument(
            "publish_livox_tf",
            default_value="true",
            description="Publish base_link -> livox_frame static TF",
        ),
        DeclareLaunchArgument(
            "base_to_livox_x",
            default_value="0.20",
            description="base_link(FLU) -> livox_frame(FRD) x [m]",
        ),
        DeclareLaunchArgument(
            "base_to_livox_y",
            default_value="0.0",
            description="base_link(FLU) -> livox_frame(FRD) y [m]",
        ),
        DeclareLaunchArgument(
            "base_to_livox_z",
            default_value="0.20",
            description="base_link(FLU) -> livox_frame(FRD) z [m]",
        ),
    ]

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key="",
            param_rewrites={
                "use_sim_time": use_sim_time,
            },
            convert_types=True,
        ),
        allow_substs=True,
    )

    # -------------------------------------------------------------------------
    # Static TF
    #
    # TF target:
    #   map
    #    └── camera_init
    #         └── livox_frame   # GenZ odometry / current scan pose
    #              └── base_link
    #
    # livox_frame(FRD) -> base_link(FLU)
    # If base_link -> livox_frame is [x, y, z] with Rx(pi),
    # then livox_frame -> base_link is [-x, y, z] with Rx(pi).
    # -------------------------------------------------------------------------
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

    livox_frame_to_base_link = static_transform_node(
        name="livox_frame_to_base_link",
        frame_id="livox_frame",
        child_frame_id="base_link",
        x=PythonExpression(["-(", base_to_livox_x, ")"]),
        y=base_to_livox_y,
        z=base_to_livox_z,
        roll=PI,
        pitch="0.0",
        yaw="0.0",
    )

    # -------------------------------------------------------------------------
    # Nav2 map server
    #
    # This is not localization. It only publishes the 2D OccupancyGrid /map
    # required by the Nav2 global costmap static layer.
    # -------------------------------------------------------------------------
    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "yaml_filename": map_yaml,
            }
        ],
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
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "autostart": autostart,
                "node_names": [
                    "map_server",
                    "planner_server",
                    "controller_server",
                    "smoother_server",
                    "behavior_server",
                    "bt_navigator",
                ],
            }
        ],
    )

    return LaunchDescription(
        declared_arguments
        + [
            # body_to_base_link,
            # base_link_to_livox_frame,
            livox_frame_to_base_link,
            map_server,
            planner_server,
            controller_server,
            smoother_server,
            behavior_server,
            bt_navigator,
            lifecycle_manager,
        ]
    )