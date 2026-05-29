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
    debug_odom = LaunchConfiguration("debug_odom")
    debug_tf = LaunchConfiguration("debug_tf")

    default_params = PathJoinSubstitution(
        [FindPackageShare("robot_base_driver"), "config", "base.yaml"]
    )

    node = Node(
        package="robot_base_driver",
        executable="robot_base_driver_node",
        name="robot_base_driver",
        namespace=namespace,
        parameters=[
            params_file,
            {
                "use_sim_time": use_sim_time,
                "debug_odom": debug_odom,
                "debug_tf": debug_tf,
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
                description="Path to the robot base YAML file.",
            ),
            DeclareLaunchArgument(
                "namespace",
                default_value="",
                description="Optional namespace for the base driver node.",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation time if true.",
            ),
            DeclareLaunchArgument(
                "log_level",
                default_value="info",
                description="ROS log level for the base driver node.",
            ),
            DeclareLaunchArgument(
                "debug_odom",
                default_value="false",
                description="Enable base odom diagnostics logs.",
            ),
            DeclareLaunchArgument(
                "debug_tf",
                default_value="false",
                description="Enable base TF diagnostics logs.",
            ),
            node,
        ]
    )
