from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sensor = LaunchConfiguration("use_sensor")
    use_motor = LaunchConfiguration("use_motor")
    use_description = LaunchConfiguration("use_description")
    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = LaunchConfiguration("params_file")
    namespace = LaunchConfiguration("namespace")
    log_level = LaunchConfiguration("log_level")
    scan_yaw_offset = LaunchConfiguration("scan_yaw_offset")
    debug_scan_geometry = LaunchConfiguration("debug_scan_geometry")

    bringup_share = FindPackageShare("robot_bringup")
    default_params = PathJoinSubstitution([bringup_share, "config", "robot.yaml"])
    description_share = FindPackageShare("robot_description")

    sensor_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([bringup_share, "launch", "sensor.launch.py"])
        ),
        condition=IfCondition(use_sensor),
        launch_arguments={
            "params_file": params_file,
            "namespace": namespace,
            "use_sim_time": use_sim_time,
            "log_level": log_level,
            "debug_scan_geometry": debug_scan_geometry,
        }.items(),
    )

    motor_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([bringup_share, "launch", "motor.launch.py"])
        ),
        condition=IfCondition(use_motor),
        launch_arguments={
            "params_file": params_file,
            "namespace": namespace,
            "use_sim_time": use_sim_time,
            "log_level": log_level,
        }.items(),
    )

    description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([description_share, "launch", "description.launch.py"])
        ),
        condition=IfCondition(use_description),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "namespace": namespace,
            "scan_yaw_offset": scan_yaw_offset,
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sensor",
                default_value="true",
                description="Launch the LiDAR bringup if true.",
            ),
            DeclareLaunchArgument(
                "use_motor",
                default_value="true",
                description="Launch the motor/base bringup if true.",
            ),
            DeclareLaunchArgument(
                "use_description",
                default_value="true",
                description="Launch robot_state_publisher and the robot description if true.",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation clock if true.",
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params,
                description="Shared parameter YAML used by the integrated bringup.",
            ),
            DeclareLaunchArgument(
                "namespace",
                default_value="",
                description="Optional namespace for robot driver nodes.",
            ),
            DeclareLaunchArgument(
                "log_level",
                default_value="info",
                description="ROS log level for included driver nodes.",
            ),
            DeclareLaunchArgument(
                "scan_yaw_offset",
                default_value="0.0",
                description="Yaw rotation from base_link to base_scan in radians.",
            ),
            DeclareLaunchArgument(
                "debug_scan_geometry",
                default_value="false",
                description="Enable LiDAR scan geometry and raw-to-scan mapping logs.",
            ),
            description_launch,
            sensor_launch,
            motor_launch,
        ]
    )
