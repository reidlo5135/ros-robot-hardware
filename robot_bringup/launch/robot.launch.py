from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sensor = LaunchConfiguration("use_sensor")
    use_motor = LaunchConfiguration("use_motor")
    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = LaunchConfiguration("params_file")
    namespace = LaunchConfiguration("namespace")
    log_level = LaunchConfiguration("log_level")

    bringup_share = FindPackageShare("robot_bringup")
    default_params = PathJoinSubstitution([bringup_share, "config", "robot.yaml"])

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
            sensor_launch,
            motor_launch,
        ]
    )
