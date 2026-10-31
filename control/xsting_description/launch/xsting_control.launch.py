from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    use_mock_base = LaunchConfiguration("use_mock_base")
    wheel_port = LaunchConfiguration("wheel_port")
    wheel_odom_tf = LaunchConfiguration("wheel_odom_tf")

    pkg_share = FindPackageShare("xsting_description")
    urdf_file = PathJoinSubstitution([pkg_share, "urdf", "mk1_xsting.urdf.xacro"])
    controllers_file = PathJoinSubstitution([pkg_share, "config", "controllers.yaml"])
    no_odom_tf_file = PathJoinSubstitution([pkg_share, "config", "diff_drive_no_odom_tf.yaml"])

    # Keep the URDF a plain string so ROS does not try to parse it as YAML.
    robot_description = ParameterValue(
        Command([
            "xacro ", urdf_file,
            " use_mock_hardware:=", use_mock_hardware,
            " use_mock_base:=", use_mock_base,
            " wheel_port:=", wheel_port,
        ]),
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

    # diff_drive_controller listens on ~/cmd_vel and publishes ~/odom, i.e.
    # /diff_drive_controller/cmd_vel. teleop and Nav2 use /cmd_vel and /odom, so remap them.
    # A remapping on ros2_control_node does not reach the controllers; it has to go through the spawner.
    diff_drive_remaps = [
        "--controller-ros-args", "-r ~/cmd_vel:=/cmd_vel",
        "--controller-ros-args", "-r ~/odom:=/odom",
    ]
    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_controller", "--param-file", controllers_file, *diff_drive_remaps],
        condition=IfCondition(wheel_odom_tf),
    )
    # wheel_odom_tf:=false: the second param file turns off odom -> base_footprint so cuVSLAM can own it.
    diff_drive_spawner_no_odom_tf = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "diff_drive_controller", "--param-file", controllers_file,
            "--param-file", no_odom_tf_file, *diff_drive_remaps,
        ],
        condition=UnlessCondition(wheel_odom_tf),
    )

    # Publishes sensor_msgs/Imu on ~/imu; remapped to /imu like cmd_vel and odom above.
    imu_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "imu_sensor_broadcaster", "--param-file", controllers_file,
            "--controller-ros-args", "-r ~/imu:=/imu",
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_mock_hardware", default_value="true"),
        DeclareLaunchArgument("use_mock_base", default_value="true"),
        DeclareLaunchArgument(
            "wheel_port",
            default_value="auto",
            description="Wheel OpenCR port; 'auto' searches /dev/ttyACM* for the board's ID",
        ),
        DeclareLaunchArgument(
            "wheel_odom_tf",
            default_value="true",
            description="Publish odom -> base_footprint from the wheels; false when cuVSLAM publishes it",
        ),
        robot_state_publisher,
        control_node,
        joint_state_broadcaster_spawner,
        controller_spawner,
        diff_drive_spawner,
        diff_drive_spawner_no_odom_tf,
        imu_broadcaster_spawner,
    ])
