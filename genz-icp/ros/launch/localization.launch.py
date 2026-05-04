from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def bool_param(name):
    return ParameterValue(LaunchConfiguration(name), value_type=bool)


def float_param(name):
    return ParameterValue(LaunchConfiguration(name), value_type=float)


def generate_launch_description():
    current_pkg = FindPackageShare("genz_icp")

    declared_arguments = [
        DeclareLaunchArgument("topic", default_value="/livox/lidar"),
        DeclareLaunchArgument("bagfile", default_value=""),
        DeclareLaunchArgument("config_file", default_value="localization_corridor.yaml"),
        DeclareLaunchArgument("map_path", default_value="/home/ams4976/ros2_ws/src/FAST_LIO_LOCALIZATION2/PCD/test.pcd"),
        DeclareLaunchArgument("map_frame", default_value="map"),
        DeclareLaunchArgument("odom_frame", default_value="camera_init"),
        DeclareLaunchArgument("base_frame", default_value="body"),       # livox_frame -> body
        DeclareLaunchArgument("publish_map_to_odom_tf", default_value="true"),
        DeclareLaunchArgument("publish_map_to_base_tf", default_value="false"),
        DeclareLaunchArgument("visualize", default_value="true"),
        DeclareLaunchArgument("require_initial_pose", default_value="true"),
        DeclareLaunchArgument("use_initial_pose_from_params", default_value="false"),
        DeclareLaunchArgument("initial_pose_x", default_value="0.0"),
        DeclareLaunchArgument("initial_pose_y", default_value="0.0"),
        DeclareLaunchArgument("initial_pose_z", default_value="0.0"),
        DeclareLaunchArgument("initial_pose_roll", default_value="0.0"),
        DeclareLaunchArgument("initial_pose_pitch", default_value="0.0"),
        DeclareLaunchArgument("initial_pose_yaw", default_value="0.0"),
        DeclareLaunchArgument(
            "rviz_config",
            default_value=PathJoinSubstitution(
                [current_pkg, "rviz", "genz_icp_localization_ros2.rviz"]
            ),
        ),
    ]

    localization_node = Node(
        package="genz_icp",
        executable="localization_node",
        name="genz_localization",
        output="screen",
        remappings=[("pointcloud_topic", LaunchConfiguration("topic"))],
        parameters=[
            {
                "config_file": LaunchConfiguration("config_file"),
                "map_path": LaunchConfiguration("map_path"),
                "map_frame": LaunchConfiguration("map_frame"),
                "odom_frame": LaunchConfiguration("odom_frame"),
                "base_frame": LaunchConfiguration("base_frame"),
                "publish_map_to_odom_tf": bool_param("publish_map_to_odom_tf"),
                "publish_map_to_base_tf": bool_param("publish_map_to_base_tf"),
                "visualize": bool_param("visualize"),
                "require_initial_pose": bool_param("require_initial_pose"),
                "use_initial_pose_from_params": bool_param("use_initial_pose_from_params"),
                "initial_pose_x": float_param("initial_pose_x"),
                "initial_pose_y": float_param("initial_pose_y"),
                "initial_pose_z": float_param("initial_pose_z"),
                "initial_pose_roll": float_param("initial_pose_roll"),
                "initial_pose_pitch": float_param("initial_pose_pitch"),
                "initial_pose_yaw": float_param("initial_pose_yaw"),
            }
        ],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output={"both": "log"},
        arguments=["-d", LaunchConfiguration("rviz_config")],
        condition=IfCondition(LaunchConfiguration("visualize")),
    )

    odometry_node = Node(
        package="genz_icp",
        executable="odometry_node",
        name="genz_odometry",
        output="screen",
        remapping=[("pointcloud_topic", LaunchConfiguration("topic"))],
        parameters=[
            {
                "config_file": LaunchConfiguration("odometry_config_file"),
                "odom_frame": LaunchConfiguration("odom_frame"),
                "base_frame": LaunchConfiguration("base_frame"),
                "publish_odom_tf": True,
                "visualize": False,
            }
        ],
    )

    bag_play = ExecuteProcess(
        cmd=["ros2", "bag", "play", LaunchConfiguration("bagfile")],
        output="screen",
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration("bagfile"), "' != ''"])
        ),
    )

    return LaunchDescription(declared_arguments + [odometry_node, localization_node, rviz_node, bag_play])
