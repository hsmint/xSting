from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _is_true(context, name):
    return LaunchConfiguration(name).perform(context).strip().lower() in (
        'true', '1', 'yes', 'on'
    )


# 런치 인자를 결합해 xSting 전방향 보행 제어 노드 및 Foxglove 브리지·URDF 실행
def build_nodes(context):
    share = Path(get_package_share_directory('xsting_walk'))
    cfg_file = LaunchConfiguration('config_file').perform(context)
    calib_file = LaunchConfiguration('calib_file').perform(context)
    dry_run = _is_true(context, 'dry_run')
    joy_driver = _is_true(context, 'joy_driver')
    joy_dev = LaunchConfiguration('joy_dev').perform(context)
    sim = _is_true(context, 'use_sim_time')
    use_foxglove = _is_true(context, 'foxglove')
    use_urdf = _is_true(context, 'urdf')
    use_static_odom = _is_true(context, 'static_odom_tf')
    fg_port = int(LaunchConfiguration('foxglove_port').perform(context))

    overrides = {
        'dry_run': dry_run,
        'use_sim_time': sim,
    }
    if calib_file:
        overrides['calib_file'] = calib_file

    nodes = [
        Node(
            package='xsting_walk',
            executable='walk_node',
            name='walk_node',
            output='screen',
            parameters=[cfg_file or str(share / 'config' / 'walk.yaml'),
                        overrides],
        )
    ]

    if joy_driver:
        nodes.append(
            Node(
                package='joy',
                executable='joy_node',
                name='joy_node',
                output='screen',
                parameters=[{
                    'device_name': joy_dev,
                    'deadzone': 0.05,
                    'autorepeat_rate': 25.0,
                }],
            )
        )

    # xsting_bringup 미실행 단독 보정 시에도 Foxglove 3D 로봇 모델 및 odom 좌표계 제공
    if use_foxglove and use_urdf:
        desc_share = Path(get_package_share_directory('xsting_description'))
        urdf_xml = (desc_share / 'urdf' / 'xsting.urdf').read_text(
            encoding='utf-8'
        )
        nodes.append(
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                parameters=[{'robot_description': urdf_xml}],
            )
        )
        if use_static_odom:
            nodes.append(
                Node(
                    package='tf2_ros',
                    executable='static_transform_publisher',
                    name='walk_static_odom_tf',
                    arguments=[
                        '--frame-id', 'odom',
                        '--child-frame-id', 'base',
                    ],
                )
            )

    if use_foxglove:
        fg_whitelist = [
            r'^(?!'
            r'.*/(?!_action/(feedback|status)$)[^A-Za-z~].*$'
            r'|.*/_buf.*$'
            r'|.*/yolo/(encoder|tensor_pub|tensor_sub|output_tensor)(/.*)?$'
            r'|/sign_yolo/(tensor_.*|output_tensor)$'
            r'|/camera/(infra1|infra2)/.*$'
            r'|/camera/(color|depth)/image.*$'
            r'|/camera/depth/metadata$'
            r'|/arducam/image_rect$'
            r'|/nvblox_node/(mesh|.*_layer|.*_map_slice|back_projected_depth/.*)$'
            r'|/nvblox_node/(tsdf|freespace|dynamic_occupancy)_layer_marker$'
            r'|/visual_slam/(status|tracking/vo_odometry)$'
            r'|/lidar/pointcloud$'
            r').*$'
        ]
        nodes.append(
            Node(
                package='foxglove_bridge',
                executable='foxglove_bridge',
                name='foxglove_bridge',
                output='screen',
                parameters=[{
                    'port': fg_port,
                    'send_buffer_limit': 15000000,
                    'min_qos_depth': 1,
                    'max_qos_depth': 5,
                    'message_backlog_size': 64,
                    'sysinfo': False,
                    'include_hidden': True,
                    'topic_whitelist': fg_whitelist,
                    'capabilities': [
                        'clientPublish',
                        'parameters',
                        'parametersSubscribe',
                        'services',
                        'connectionGraph',
                        'assets',
                    ],
                }],
            )
        )
    return nodes


def generate_launch_description():
    share = Path(get_package_share_directory('xsting_walk'))
    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file', default_value=str(share / 'config' / 'walk.yaml')
        ),
        DeclareLaunchArgument('calib_file', default_value=''),
        DeclareLaunchArgument('dry_run', default_value='false'),
        DeclareLaunchArgument('joy_driver', default_value='false'),
        DeclareLaunchArgument('joy_dev', default_value=''),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('foxglove', default_value='false'),
        DeclareLaunchArgument('foxglove_port', default_value='8765'),
        DeclareLaunchArgument('urdf', default_value='true'),
        DeclareLaunchArgument('static_odom_tf', default_value='true'),
        OpaqueFunction(function=build_nodes),
    ])
