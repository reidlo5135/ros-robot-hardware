from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = LaunchConfiguration("params_file")
    namespace = LaunchConfiguration("namespace")
    use_sim_time = LaunchConfiguration("use_sim_time")
    enabled = LaunchConfiguration("enabled")
    log_level = LaunchConfiguration("log_level")

    default_params = PathJoinSubstitution(
        [FindPackageShare("robot_bringup"), "config", "robot.yaml"]
    )

    bms_node = Node(
        package="robot_bms_driver",
        executable="robot_bms_driver_node",
        name="robot_bms_driver",
        namespace=namespace,
        condition=IfCondition(enabled),
        parameters=[
            params_file,
            {
                "use_sim_time": use_sim_time,
                "bms.enabled": enabled,
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
                description="Optional namespace for the BMS node.",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation clock if true.",
            ),
            DeclareLaunchArgument(
                "enabled",
                default_value="false",
                description="Enable BMS serial polling if true.",
            ),
            DeclareLaunchArgument(
                "log_level",
                default_value="info",
                description="ROS log level for the BMS driver node.",
            ),
            bms_node,
        ]
    )