from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = LaunchConfiguration("params_file")
    namespace = LaunchConfiguration("namespace")
    use_sim_time = LaunchConfiguration("use_sim_time")
    log_level = LaunchConfiguration("log_level")
    debug_scan_geometry = LaunchConfiguration("debug_scan_geometry")

    default_params = PathJoinSubstitution(
        [FindPackageShare("robot_bringup"), "config", "robot.yaml"]
    )

    lidar_node = Node(
        package="robot_lidar_driver",
        executable="lidar_driver_node",
        name="robot_lidar_driver",
        namespace=namespace,
        parameters=[
            params_file,
            {
                "use_sim_time": use_sim_time,
                "debug_scan_geometry": debug_scan_geometry,
            },
        ],
        arguments=["--ros-args", "--log-level", log_level],
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params,
                description="Path to the integrated robot parameter YAML file.",
            ),
            DeclareLaunchArgument(
                "namespace",
                default_value="",
                description="Optional namespace for the LiDAR node.",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation clock if true.",
            ),
            DeclareLaunchArgument(
                "log_level",
                default_value="info",
                description="ROS log level for the LiDAR driver node.",
            ),
            DeclareLaunchArgument(
                "debug_scan_geometry",
                default_value="false",
                description="Enable LiDAR scan geometry and raw-to-scan mapping logs.",
            ),
            lidar_node,
        ]
    )
