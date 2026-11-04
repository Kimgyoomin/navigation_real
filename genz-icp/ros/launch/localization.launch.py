from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from navigation_dds.profiles import dds_argument, dds_group


def bool_param(name):
    return ParameterValue(LaunchConfiguration(name), value_type=bool)


def float_param(name):
    return ParameterValue(LaunchConfiguration(name), value_type=float)


def mount_transform(context):
    # Sensor-origin tests must not give livox_frame a second parent (or itself).
    base_frame = LaunchConfiguration("base_frame").perform(context)
    lidar_frame = LaunchConfiguration("lidar_frame").perform(context)
    if not base_frame or not lidar_frame:
        raise RuntimeError("base_frame and lidar_frame must be nonempty")
    if base_frame == lidar_frame:
        return []
    if not bool_param("publish_livox_tf").evaluate(context):
        return []
    return [Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_link_to_livox_frame",
        output="screen",
        arguments=[
            "--x", LaunchConfiguration("base_to_livox_x"),
            "--y", LaunchConfiguration("base_to_livox_y"),
            "--z", LaunchConfiguration("base_to_livox_z"),
            "--roll", LaunchConfiguration("base_to_livox_roll"),
            "--pitch", LaunchConfiguration("base_to_livox_pitch"),
            "--yaw", LaunchConfiguration("base_to_livox_yaw"),
            "--frame-id", base_frame,
            "--child-frame-id", lidar_frame,
        ],
        parameters=[{"use_sim_time": bool_param("use_sim_time")}],
    )]


def generate_launch_description():
    current_pkg = FindPackageShare("genz_icp")

    declared_arguments = [
        dds_argument(),
        # Input
        DeclareLaunchArgument(
            "topic", default_value="/livox/lidar_pointcloud",
            description="sensor_msgs/PointCloud2 input; /livox/lidar is CustomMsg.",
        ),
        
        # Time policy
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use /clock. Set true only for rosbag/simulation replay.",
        ),
        DeclareLaunchArgument(
            "use_sensor_stamp",
            default_value="true",
            description=(
                "If true, publish odom/TF/localization outputs using PointCloud2 header.stamp. "
                "If false, publish using node now()."
            ),
        ),

        # GenZ-ICP config
        # Only localization_rosbag.launch.py owns bag playback.
        DeclareLaunchArgument("config_file", default_value="localization_corridor.yaml"),
        DeclareLaunchArgument("map_path", default_value="/home/rclab/ros2_ws/src/navigation_real/FAST_LIO_LOCALIZATION2/PCD/1f_fast_lio_260731_segmented.pcd"),

        # Frame policy
        DeclareLaunchArgument("map_frame", default_value="map"),
        DeclareLaunchArgument("odom_frame", default_value="camera_init"),
        DeclareLaunchArgument("base_frame", default_value="base_link"),
        DeclareLaunchArgument("lidar_frame", default_value="livox_frame"),
        DeclareLaunchArgument("publish_livox_tf", default_value="true"),
        DeclareLaunchArgument("base_to_livox_x", default_value="0.0995"),
        DeclareLaunchArgument("base_to_livox_y", default_value="0.0"),
        DeclareLaunchArgument("base_to_livox_z", default_value="-0.1375"),
        # RUBI's driver already rotates the inverted sensor data about X.
        # These angles describe the published livox_frame, not the raw sensor.
        DeclareLaunchArgument("base_to_livox_roll", default_value="0.0"),
        DeclareLaunchArgument("base_to_livox_pitch", default_value="0.0"),
        DeclareLaunchArgument("base_to_livox_yaw", default_value="0.0"),

        # Localization TF policy
        DeclareLaunchArgument("publish_map_to_odom_tf", default_value="true"),
        DeclareLaunchArgument("publish_map_to_base_tf", default_value="false"),
        DeclareLaunchArgument(
            "transform_publish_tolerance",
            default_value="0.0",
            description="map->odom TF time offset in seconds; 0 preserves observation time.",
        ),
        
        # Debug / Rviz
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
                "use_sim_time": bool_param("use_sim_time"),
                "use_sensor_stamp": bool_param("use_sensor_stamp"),
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
                "transform_publish_tolerance": float_param("transform_publish_tolerance"),
            }
        ],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output={"both": "log"},
        arguments=["-d", LaunchConfiguration("rviz_config")],
        parameters=[{"use_sim_time": bool_param("use_sim_time")}],
        condition=IfCondition(LaunchConfiguration("visualize")),
    )

    odometry_node = Node(
        package="genz_icp",
        executable="odometry_node",
        name="genz_odometry",
        output="screen",
        remappings=[("pointcloud_topic", LaunchConfiguration("topic"))],
        parameters=[
            {   
                "use_sim_time": bool_param("use_sim_time"),
                "use_sensor_stamp": bool_param("use_sensor_stamp"),
                "config_file": LaunchConfiguration("config_file"),
                "odom_frame": LaunchConfiguration("odom_frame"),
                "base_frame": LaunchConfiguration("base_frame"),
                "publish_odom_tf": True,
                "visualize": False,
            }
        ],
    )

    return LaunchDescription(declared_arguments + [
        dds_group([
            OpaqueFunction(function=mount_transform),
            odometry_node,
            rviz_node,
            dds_group([localization_node], "GenZ PCD localization", profile="large"),
        ], "GenZ localization"),
    ])
