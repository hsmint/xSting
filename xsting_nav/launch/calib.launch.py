from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory('xsting_nav'))
    default_calib = str(share / 'config' / 'calibration.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='status'),
        DeclareLaunchArgument('calib_file', default_value=default_calib),
        DeclareLaunchArgument('hardware_profile', default_value=''),
        DeclareLaunchArgument('duration_s', default_value='3.0'),
        DeclareLaunchArgument('board_yaw_deg', default_value='0.0'),
        DeclareLaunchArgument('ref_pose', default_value='zero'),
        DeclareLaunchArgument('test_joint_index', default_value='0'),
        DeclareLaunchArgument('nudge_deg', default_value='6.0'),
        DeclareLaunchArgument('target_arena_x', default_value='0.734'),
        DeclareLaunchArgument('target_arena_y', default_value='0.950'),
        DeclareLaunchArgument('target_arena_yaw_deg', default_value='180.0'),
        DeclareLaunchArgument('gait_axis', default_value='vx'),
        DeclareLaunchArgument('gait_test_speed', default_value='0.16'),
        DeclareLaunchArgument('measured_delta', default_value='0.0'),
        DeclareLaunchArgument('known_front_wall_m', default_value='0.0'),
        Node(
            package='xsting_nav',
            executable='calib_node',
            name='calib_node',
            output='screen',
            parameters=[{
                'mode': LaunchConfiguration('mode'),
                'calib_file': LaunchConfiguration('calib_file'),
                'hardware_profile': LaunchConfiguration('hardware_profile'),
                'duration_s': LaunchConfiguration('duration_s'),
                'board_yaw_deg': LaunchConfiguration('board_yaw_deg'),
                'ref_pose': LaunchConfiguration('ref_pose'),
                'test_joint_index': LaunchConfiguration('test_joint_index'),
                'nudge_deg': LaunchConfiguration('nudge_deg'),
                'target_arena_x': LaunchConfiguration('target_arena_x'),
                'target_arena_y': LaunchConfiguration('target_arena_y'),
                'target_arena_yaw_deg': LaunchConfiguration('target_arena_yaw_deg'),
                'gait_axis': LaunchConfiguration('gait_axis'),
                'gait_test_speed': LaunchConfiguration('gait_test_speed'),
                'measured_delta': LaunchConfiguration('measured_delta'),
                'known_front_wall_m': LaunchConfiguration('known_front_wall_m'),
            }],
        ),
    ])
