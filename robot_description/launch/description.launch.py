from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    model = LaunchConfiguration("model")
    namespace = LaunchConfiguration("namespace")
    scan_yaw_offset = LaunchConfiguration("scan_yaw_offset")
    normalized_xacro_namespace = PythonExpression(
        ['"', namespace, '".strip("/") + "/" if "', namespace, '".strip("/") != "" else ""']
    )

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
                " ",
                "namespace:=",
                normalized_xacro_namespace,
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

    static_base_frame_log = LogInfo(
        msg=[
            "ROBOT_HW_LOG schema=v1 tag=TF component=description event=static_frame_config node=robot_state_publisher namespace=",
            namespace,
            " parent_frame=",
            normalized_xacro_namespace,
            "base_footprint child_frame=",
            normalized_xacro_namespace,
            "base_link x=0.000 y=0.000 z=0.010 roll_rad=0.000 pitch_rad=0.000 yaw_rad=0.000 source=robot_description result=configured",
        ]
    )
    static_scan_frame_log = LogInfo(
        msg=[
            "ROBOT_HW_LOG schema=v1 tag=TF component=description event=static_frame_config node=robot_state_publisher namespace=",
            namespace,
            " parent_frame=",
            normalized_xacro_namespace,
            "base_link child_frame=",
            normalized_xacro_namespace,
            "base_scan x=-0.032 y=0.000 z=0.172 roll_rad=0.000 pitch_rad=0.000 yaw_rad=",
            scan_yaw_offset,
            " source=robot_description result=configured",
        ]
    )
    static_imu_frame_log = LogInfo(
        msg=[
            "ROBOT_HW_LOG schema=v1 tag=TF component=description event=static_frame_config node=robot_state_publisher namespace=",
            namespace,
            " parent_frame=",
            normalized_xacro_namespace,
            "base_link child_frame=",
            normalized_xacro_namespace,
            "imu_link x=-0.032 y=0.000 z=0.068 roll_rad=0.000 pitch_rad=0.000 yaw_rad=0.000 source=robot_description result=configured",
        ]
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
            static_base_frame_log,
            static_scan_frame_log,
            static_imu_frame_log,
            robot_state_publisher,
        ]
    )
