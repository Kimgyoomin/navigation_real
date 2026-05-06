from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    current_pkg = FindPackageShare("genz_icp")

    declared_arguments = [
        # Set this default_value to your frequently used bag path if you want no-argument replay.
        DeclareLaunchArgument("bagfile", default_value=""),

        # Input topic in the bag
        DeclareLaunchArgument("topic", default_value="/livox/lidar"),

        # Bag replay clock rate. ros2 bag play <bag> --clock 40 is the default style.
        DeclareLaunchArgument("clock_rate", default_value="40"),

        # GenZ-ICP config
        DeclareLaunchArgument("config_file", default_value="localization_corridor.yaml"),
        DeclareLaunchArgument(
            "map_path",
            default_value="/home/ams4976/ros2_ws/src/FAST_LIO_LOCALIZATION2/PCD/test.pcd",
        ),

        # Frame policy
        DeclareLaunchArgument("map_frame", default_value="map"),
        DeclareLaunchArgument("odom_frame", default_value="camera_init"),
        DeclareLaunchArgument("base_frame", default_value="livox_frame"),

        # Initial pose / RViz
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

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([current_pkg, "launch", "localization.launch.py"])
        ),
        launch_arguments={
            "topic": LaunchConfiguration("topic"),
            "use_sim_time": "true",
            "use_sensor_stamp": "true",
            "config_file": LaunchConfiguration("config_file"),
            "map_path": LaunchConfiguration("map_path"),
            "map_frame": LaunchConfiguration("map_frame"),
            "odom_frame": LaunchConfiguration("odom_frame"),
            "base_frame": LaunchConfiguration("base_frame"),
            "publish_map_to_odom_tf": "true",
            "publish_map_to_base_tf": "false",
            "visualize": LaunchConfiguration("visualize"),
            "require_initial_pose": LaunchConfiguration("require_initial_pose"),
            "use_initial_pose_from_params": LaunchConfiguration("use_initial_pose_from_params"),
            "initial_pose_x": LaunchConfiguration("initial_pose_x"),
            "initial_pose_y": LaunchConfiguration("initial_pose_y"),
            "initial_pose_z": LaunchConfiguration("initial_pose_z"),
            "initial_pose_roll": LaunchConfiguration("initial_pose_roll"),
            "initial_pose_pitch": LaunchConfiguration("initial_pose_pitch"),
            "initial_pose_yaw": LaunchConfiguration("initial_pose_yaw"),
            "rviz_config": LaunchConfiguration("rviz_config"),
        }.items(),
    )

    bag_play = ExecuteProcess(
        cmd=[
            "ros2",
            "bag",
            "play",
            LaunchConfiguration("bagfile"),
            "--clock",
            LaunchConfiguration("clock_rate"),
        ],
        output="screen",
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration("bagfile"), "' != ''"])
        ),
    )

    return LaunchDescription(
        declared_arguments
        + [
            localization,
            bag_play,
        ]
    )