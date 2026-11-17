from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# 워크스페이스 소스 또는 install 공유 폴더의 calibration.yaml 절대경로 탐색
def default_calib_path() -> str:
    share = Path(get_package_share_directory('xsting_approach')).resolve()
    for root in [Path.cwd(), *share.parents]:
        cand = root / 'src' / 'xsting_approach' / 'config' / 'calibration.yaml'
        if cand.parent.exists():
            return str(cand)
    return str(share / 'config' / 'calibration.yaml')

# 대용량 원본 영상·내부 텐서를 제외한 Foxglove 허용 토픽 정규식
FG_WHITELIST = [
    r'^(?!'
    r'.*/(?!_action/(feedback|status)$)[^A-Za-z~].*$'
    r'|.*/yolo/(encoder|tensor_pub|tensor_sub|output_tensor)(/.*)?$'
    r'|/sign_yolo/(tensor_.*|output_tensor)$'
    r'|/camera/(infra1|infra2)/.*$'
    r'|/camera/(color|depth)/image.*$'
    r'|/camera/depth/metadata$'
    r'|/arducam/image_rect$'
    r'|/nvblox_node/(mesh|.*_layer|.*_map_slice|back_projected_depth/.*)$'
    r'|/lidar/pointcloud$'
    r').*$'
]


# 선택한 단계(cam / imu / joints) 및 Foxglove 브리지 노드 실행
def build_nodes(context):
    mode = LaunchConfiguration('mode').perform(context)
    calib_file = LaunchConfiguration('calib_file').perform(context)
    pose = LaunchConfiguration('pose').perform(context)
    gap_m = float(LaunchConfiguration('box_gap_m').perform(context))
    live = LaunchConfiguration('live').perform(context).lower() == 'true'
    two_pose = (
        LaunchConfiguration('two_pose').perform(context).lower() == 'true'
    )
    stand_legs = (
        LaunchConfiguration('stand_legs').perform(context).lower() == 'true'
    )
    use_fg = LaunchConfiguration('foxglove').perform(context).lower() == 'true'

    actions = []
    if mode == 'joints':
        actions.append(Node(
            package='xsting_approach',
            executable='calib_joints_node',
            output='screen',
            parameters=[{'calib_file': calib_file, 'pose': pose}],
        ))
    elif mode == 'imu':
        actions.append(Node(
            package='xsting_approach',
            executable='calib_imu_node',
            output='screen',
            parameters=[{
                'calib_file': calib_file,
                'two_pose': two_pose,
                'stand_legs': stand_legs,
            }],
        ))
    else:
        actions.append(Node(
            package='xsting_approach',
            executable='calib_cam_node',
            output='screen',
            parameters=[{
                'calib_file': calib_file,
                'box_gap_m': gap_m,
                'live': live or use_fg,
                'stand_legs': stand_legs,
            }],
        ))

    if use_fg:
        actions.append(Node(
            package='foxglove_bridge',
            executable='foxglove_bridge',
            name='foxglove_bridge',
            parameters=[{
                'port': 8765,
                'send_buffer_limit': 15000000,
                'min_qos_depth': 1,
                'max_qos_depth': 1,
                'message_backlog_size': 32,
                'sysinfo': False,
                'include_hidden': True,
                'capabilities': ['clientPublish', 'services',
                                 'connectionGraph', 'assets'],
                'service_whitelist': [r'^/(calib|approach)/.*$'],
                'topic_whitelist': FG_WHITELIST,
            }],
            output='screen',
        ))
    return actions


# 실기 보정 런치 인자 선언
def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'mode', default_value='cam', choices=['joints', 'imu', 'cam']
        ),
        DeclareLaunchArgument(
            'calib_file',
            default_value=default_calib_path(),
        ),
        DeclareLaunchArgument(
            'pose', default_value='boot', choices=['boot', 'stand', 'zero']
        ),
        DeclareLaunchArgument('box_gap_m', default_value='0.200'),
        DeclareLaunchArgument('live', default_value='true'),
        DeclareLaunchArgument('two_pose', default_value='false'),
        DeclareLaunchArgument('stand_legs', default_value='true'),
        DeclareLaunchArgument('foxglove', default_value='false'),
        OpaqueFunction(function=build_nodes),
    ])
