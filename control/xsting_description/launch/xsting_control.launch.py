from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")

    pkg_share = FindPackageShare("xsting_description")
    urdf_file = PathJoinSubstitution([pkg_share, "urdf", "mk1_xsting.urdf.xacro"])
    controllers_file = PathJoinSubstitution([pkg_share, "config", "controllers.yaml"])

    # Keep the URDF a plain string so ROS does not try to parse it as YAML.
    robot_description = ParameterValue(
        Command(["xacro ", urdf_file, " use_mock_hardware:=", use_mock_hardware]),
        value_type=str,
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[controllers_file],
        output="screen",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["position_controller", "--param-file", controllers_file],
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_mock_hardware", default_value="true"),
        robot_state_publisher,
        control_node,
        joint_state_broadcaster_spawner,
        controller_spawner,
    ])
