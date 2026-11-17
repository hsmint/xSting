"""xSting 다리, 팔 하드웨어와 컨트롤러를 옵션에 맞춰 실행한다."""

import os

from ament_index_python.packages import PackageNotFoundError
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# 앞다리 보드(ID 200) 모터 순서: 1..3(leg1), 4..6(leg2), 10..12(leg4)
FRONT_JOINTS = [
    ('leg1_yaw', 1.5707963),
    ('leg1_hip', 0.0),
    ('leg1_knee', 0.0),
    ('leg2_yaw', 1.5707963),
    ('leg2_hip', 0.0),
    ('leg2_knee', 0.0),
    ('leg4_yaw', 1.5707963),
    ('leg4_hip', 0.0),
    ('leg4_knee', 0.0),
]

# 뒷다리 보드(ID 201) 모터 순서: 7..9(leg3), 13..15(leg5), 16..18(leg6)
REAR_JOINTS = [
    ('leg3_yaw', -1.5707963),
    ('leg3_hip', 0.0),
    ('leg3_knee', 0.0),
    ('leg5_yaw', -1.5707963),
    ('leg5_hip', 0.0),
    ('leg5_knee', 0.0),
    ('leg6_yaw', -1.5707963),
    ('leg6_hip', 0.0),
    ('leg6_knee', 0.0),
]

# 팔 보드(ID 202) 모터 순서: 11(yaw), 12(shoulder), 13(elbow), 14(wrist), 15(roll), 16(finger_l)
ARM_JOINTS = [
    ('arm_yaw', 0.0),
    ('arm_shoulder', 0.0),
    ('arm_elbow', 0.1570796),
    ('arm_wrist', 0.0),
    ('arm_roll', 0.0),
    ('finger_l', 0.0162),
]

IMU_STATES = [
    'orientation.w', 'orientation.x', 'orientation.y', 'orientation.z',
    'angular_velocity.x', 'angular_velocity.y', 'angular_velocity.z',
    'linear_acceleration.x', 'linear_acceleration.y', 'linear_acceleration.z',
]


def is_true(context, name):
    val = LaunchConfiguration(name).perform(context).strip().lower()
    return val in ('true', '1', 'yes', 'on')


def joint_xml(name, off=0.0, extra_cmd=''):
    """관절 하나의 방향·오프셋 파라미터 및 명령·상태 인터페이스."""
    return (
        f'<joint name="{name}">'
        '<param name="direction">1</param>'
        f'<param name="offset">{off}</param>'
        '<command_interface name="position"/>'
        f'{extra_cmd}'
        '<state_interface name="position"/>'
        '<state_interface name="velocity"/>'
        '</joint>'
    )


def ros2_control_xml(use_leg, use_arm, stand, reboot_leg, reboot_arm):
    """옵션에 맞는 <ros2_control> 블록."""
    b_leg = 'true' if use_leg else 'false'
    b_arm = 'true' if use_arm else 'false'
    b_stand = 'true' if stand else 'false'
    r_leg = 'true' if reboot_leg else 'false'
    r_arm = 'true' if reboot_arm else 'false'

    xml = (
        '<ros2_control name="xsting" type="system"><hardware>'
        '<plugin>xsting_hardware/XstingSystem</plugin>'
        f'<param name="use_leg">{b_leg}</param>'
        f'<param name="use_arm">{b_arm}</param>'
        f'<param name="stand">{b_stand}</param>'
        f'<param name="reboot_leg">{r_leg}</param>'
        f'<param name="reboot_arm">{r_arm}</param>'
        '</hardware>'
    )

    if use_leg:
        for name, off in FRONT_JOINTS + REAR_JOINTS:
            xml += joint_xml(name, off)
        states = ''.join(f'<state_interface name="{s}"/>' for s in IMU_STATES)
        xml += f'<sensor name="imu_front">{states}</sensor>'
        xml += '<sensor name="ultrasonic_front"><state_interface name="range"/></sensor>'

    if use_arm:
        for name, off in ARM_JOINTS[:-1]:
            xml += joint_xml(name, off)
        g_name, g_off = ARM_JOINTS[-1]
        xml += joint_xml(g_name, g_off, '<command_interface name="effort"/>')
        xml += (
            '<joint name="finger_r">'
            '<state_interface name="position"/>'
            '<state_interface name="velocity"/>'
            '</joint>'
        )

    return xml + '</ros2_control>'


def fallback_urdf(use_leg, use_arm):
    """외부 URDF 패키지가 없을 때 사용하는 최소 URDF 골격."""
    xml = '<?xml version="1.0"?><robot name="xsting"><link name="base"/>'
    if use_leg:
        xml += (
            '<link name="imu"/>'
            '<joint name="imu_joint" type="fixed">'
            '<parent link="base"/><child link="imu"/></joint>'
            '<link name="ultrasonic"/>'
            '<joint name="ultrasonic_joint" type="fixed">'
            '<parent link="base"/><child link="ultrasonic"/></joint>'
        )
        for name, _ in FRONT_JOINTS + REAR_JOINTS:
            xml += (
                f'<link name="{name}"/>'
                f'<joint name="{name}" type="continuous">'
                f'<parent link="base"/><child link="{name}"/>'
                '<axis xyz="0 0 1"/></joint>'
            )
    if use_arm:
        for name, _ in ARM_JOINTS[:-1]:
            xml += (
                f'<link name="{name}"/>'
                f'<joint name="{name}" type="continuous">'
                f'<parent link="base"/><child link="{name}"/>'
                '<axis xyz="0 0 1"/></joint>'
            )
        xml += (
            '<link name="finger_l"/>'
            '<joint name="finger_l" type="prismatic">'
            '<parent link="base"/><child link="finger_l"/>'
            '<axis xyz="0 -1 0"/>'
            '<limit lower="-0.00186" upper="0.025" effort="10" velocity="0.05"/>'
            '</joint>'
            '<link name="finger_r"/>'
            '<joint name="finger_r" type="prismatic">'
            '<parent link="base"/><child link="finger_r"/>'
            '<axis xyz="0 1 0"/>'
            '<limit lower="-0.00186" upper="0.025" effort="10" velocity="0.05"/>'
            '<mimic joint="finger_l" multiplier="1" offset="0"/>'
            '</joint>'
        )
    return xml + '</robot>'


def load_base_urdf(use_leg, use_arm):
    """설치된 xSting URDF가 있으면 불러오고, 없으면 최소 URDF로 폴백한다."""
    try:
        urdf_path = os.path.join(
            get_package_share_directory('xsting_description'),
            'urdf',
            'xsting.urdf',
        )
        with open(urdf_path, encoding='utf-8') as f:
            return f.read()
    except (PackageNotFoundError, FileNotFoundError):
        return fallback_urdf(use_leg, use_arm)


def launch_setup(context):
    use_leg = is_true(context, 'use_leg')
    use_arm = is_true(context, 'use_arm')
    stand = is_true(context, 'stand')
    reboot_all = is_true(context, 'reboot')
    reboot_leg = reboot_all or is_true(context, 'reboot_leg')
    reboot_arm = reboot_all or is_true(context, 'reboot_arm')

    ctrl_xml = ros2_control_xml(use_leg, use_arm, stand, reboot_leg, reboot_arm)
    description = load_base_urdf(use_leg, use_arm).replace(
        '</robot>', ctrl_xml + '</robot>'
    )

    controllers = []
    if use_leg or use_arm:
        controllers.append('joint_state_broadcaster')
    if use_leg:
        controllers += ['leg_controller', 'imu_broadcaster', 'sonar_broadcaster']
    if use_arm:
        controllers += ['arm_controller']

    config = os.path.join(
        get_package_share_directory('xsting_hardware'),
        'config',
        'xsting_controllers.yaml',
    )

    actions = [
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': description}],
        ),
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[config],
            output='screen',
            sigterm_timeout='15',
        ),
    ]
    if controllers:
        actions.append(
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=controllers + ['--param-file', config],
            )
        )
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_leg', default_value='true'),
        DeclareLaunchArgument('use_arm', default_value='true'),
        DeclareLaunchArgument('stand', default_value='true'),
        DeclareLaunchArgument('reboot', default_value='false'),
        DeclareLaunchArgument('reboot_leg', default_value='false'),
        DeclareLaunchArgument('reboot_arm', default_value='false'),
        OpaqueFunction(function=launch_setup),
    ])
