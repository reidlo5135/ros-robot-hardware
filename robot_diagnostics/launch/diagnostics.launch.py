from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    contract_file = LaunchConfiguration("contract_file")
    summary_period_sec = LaunchConfiguration("summary_period_sec")
    once = LaunchConfiguration("once")
    log_level = LaunchConfiguration("log_level")

    default_contract = PathJoinSubstitution(
        [FindPackageShare("robot_diagnostics"), "config", "tb3_contract.yaml"]
    )

    node = Node(
        package="robot_diagnostics",
        executable="robot_diagnostics_node",
        name="robot_diagnostics",
        parameters=[
            {
                "contract_file": contract_file,
                "summary_period_sec": summary_period_sec,
                "once": once,
            }
        ],
        arguments=["--ros-args", "--log-level", log_level],
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "contract_file",
                default_value=default_contract,
                description="Path to the robot hardware contract YAML file.",
            ),
            DeclareLaunchArgument(
                "summary_period_sec",
                default_value="2.0",
                description="Seconds between PASS/WARN/FAIL diagnostic summaries.",
            ),
            DeclareLaunchArgument(
                "once",
                default_value="false",
                description="Run one summary cycle and exit if true.",
            ),
            DeclareLaunchArgument(
                "log_level",
                default_value="info",
                description="ROS log level for the diagnostics node.",
            ),
            node,
        ]
    )