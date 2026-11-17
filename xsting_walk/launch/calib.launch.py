from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# 선택한 단계(imu / joints / gait)에 맞는 실측 보정 노드 실행
def build_nodes(context):
    mode = LaunchConfiguration('mode').perform(context)
    calib_file = LaunchConfiguration('calib_file').perform(context)
    pose = LaunchConfiguration('pose').perform(context)
    action = LaunchConfiguration('action').perform(context)
    two_pose = (
        LaunchConfiguration('two_pose').perform(context).lower() == 'true'
    )
    axis = LaunchConfiguration('axis').perform(context)
    speed = float(LaunchConfiguration('speed').perform(context))
    duration_s = float(LaunchConfiguration('duration_s').perform(context))

    if mode == 'joints':
        return [Node(
            package='xsting_walk',
            executable='calib_joints_node',
            output='screen',
            parameters=[{
                'calib_file': calib_file,
                'action': action,
                'pose': pose,
            }],
        )]
    if mode == 'imu':
        return [Node(
            package='xsting_walk',
            executable='calib_imu_node',
            output='screen',
            parameters=[{'calib_file': calib_file, 'two_pose': two_pose}],
        )]
    return [Node(
        package='xsting_walk',
        executable='calib_gait_node',
        output='screen',
        parameters=[{
            'calib_file': calib_file,
            'mode': 'trial',
            'axis': axis,
            'speed': speed,
            'duration_s': duration_s,
            'auto_apply_sensor': True,
        }],
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'mode', default_value='imu', choices=['imu', 'joints', 'gait']
        ),
        DeclareLaunchArgument(
            'calib_file',
            default_value='src/xsting_walk/config/calibration.yaml',
        ),
        DeclareLaunchArgument(
            'action', default_value='offset',
            choices=['offset', 'stand', 'nudge', 'flip_sign'],
        ),
        DeclareLaunchArgument(
            'pose', default_value='boot', choices=['boot', 'stand', 'zero']
        ),
        DeclareLaunchArgument('two_pose', default_value='false'),
        DeclareLaunchArgument(
            'axis', default_value='vx', choices=['vx', 'vy', 'wz']
        ),
        DeclareLaunchArgument('speed', default_value='0.20'),
        DeclareLaunchArgument('duration_s', default_value='3.0'),
        OpaqueFunction(function=build_nodes),
    ])
