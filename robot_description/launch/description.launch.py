from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    model = LaunchConfiguration("model")
    namespace = LaunchConfiguration("namespace")
    scan_yaw_offset = LaunchConfiguration("scan_yaw_offset")

    default_model = PathJoinSubstitution(
        [FindPackageShare("robot_description"), "urdf", "robot.urdf.xacro"]
    )

    robot_description = ParameterValue(
        Command(
            [
                FindExecutable(name="xacro"),
                " ",
                model,
                " ",
                "scan_yaw_offset:=",
                scan_yaw_offset,
            ]
        ),
        value_type=str,
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        namespace=namespace,
        output="screen",
        parameters=[
            {
                "robot_description": robot_description,
                "use_sim_time": use_sim_time,
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation clock if true.",
            ),
            DeclareLaunchArgument(
                "model",
                default_value=default_model,
                description="Absolute path to the robot URDF xacro file.",
            ),
            DeclareLaunchArgument(
                "namespace",
                default_value="",
                description="Optional namespace for robot_state_publisher.",
            ),
            DeclareLaunchArgument(
                "scan_yaw_offset",
                default_value="0.0",
                description="Yaw rotation from base_link to base_scan in radians.",
            ),
            robot_state_publisher,
        ]
    )
