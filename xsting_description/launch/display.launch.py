"""xSting URDF를 RViz에 띄우고 관절 슬라이더로 확인하는 런치 파일.

실행: ros2 launch xsting_description display.launch.py [pose:=stand|zero] [gui:=true|false]
"""

import importlib.util
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _setup(context):
    share_dir = get_package_share_directory('xsting_description')
    urdf_path = os.path.join(share_dir, 'urdf', 'xsting.urdf')
    rviz_path = os.path.join(share_dir, 'rviz', 'display.rviz')
    order_path = os.path.join(share_dir, 'config', 'joint_order.yaml')

    with open(urdf_path, 'r', encoding='utf-8') as f:
        robot_description = f.read()

    zeros = {}
    if LaunchConfiguration('pose').perform(context).strip().lower() == 'stand':
        with open(order_path, 'r', encoding='utf-8') as f:
            order = yaml.safe_load(f)
        zeros = {
            j['name']: float(j['default'])
            for v in order.values() if isinstance(v, list)
            for j in v if isinstance(j, dict) and 'default' in j
        }

    use_gui = LaunchConfiguration('gui').perform(context).strip().lower() in (
        'true', '1', 'yes', 'on'
    )
    if use_gui and not any(
        importlib.util.find_spec(m) for m in ('PyQt5', 'PySide2', 'PySide6')
    ):
        use_gui = False

    jsp_pkg = 'joint_state_publisher_gui' if use_gui else 'joint_state_publisher'
    jsp_params = [{'zeros': zeros}] if zeros else []

    return [
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
        ),
        Node(
            package=jsp_pkg,
            executable=jsp_pkg,
            parameters=jsp_params,
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_path],
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'pose',
            default_value='stand',
            description='stand = 기본 서기·팔 접힘 자세, zero = 모든 관절 0 rad (L-pose)',
        ),
        DeclareLaunchArgument(
            'gui',
            default_value='true',
            description='관절 슬라이더 GUI(joint_state_publisher_gui) 실행 여부',
        ),
        OpaqueFunction(function=_setup),
    ])
