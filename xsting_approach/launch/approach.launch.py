from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

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


# 워크스페이스 소스 calibration.yaml이 존재하면 최우선 로드하도록 경로 탐색
def default_calib_path(share: Path) -> str:
    for root in [Path.cwd(), *share.resolve().parents]:
        cand = root / 'src' / 'xsting_approach' / 'config' / 'calibration.yaml'
        if cand.exists():
            return str(cand)
    return str(share / 'config' / 'calibration.yaml')


# 런치 인자를 결합해 정밀 접근 노드 및 선택적 Foxglove 브리지 실행
def build_nodes(context):
    share = Path(get_package_share_directory('xsting_approach'))
    cfg_file = LaunchConfiguration('config_file').perform(context)
    calib_file = LaunchConfiguration('calib_file').perform(context)
    dry_run = LaunchConfiguration('dry_run').perform(context).lower() == 'true'
    auto_start = (
        LaunchConfiguration('auto_start').perform(context).lower() == 'true'
    )
    stand_when_idle = (
        LaunchConfiguration('stand_when_idle').perform(context).lower() == 'true'
    )
    sim = LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'
    use_fg = LaunchConfiguration('foxglove').perform(context).lower() == 'true'

    overrides = {
        'calib_file': calib_file or default_calib_path(share),
        'dry_run': dry_run,
        'auto_start': auto_start,
        'stand_when_idle': stand_when_idle,
        'use_sim_time': sim,
    }

    actions = [
        Node(
            package='xsting_approach',
            executable='approach_node',
            name='approach_node',
            output='screen',
            parameters=[cfg_file or str(share / 'config' / 'approach.yaml'),
                        overrides],
        )
    ]

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


# 정밀 접근 런치 인자 선언
def generate_launch_description():
    share = Path(get_package_share_directory('xsting_approach'))
    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file', default_value=str(share / 'config' / 'approach.yaml')
        ),
        DeclareLaunchArgument('calib_file', default_value=''),
        DeclareLaunchArgument('dry_run', default_value='false'),
        DeclareLaunchArgument('auto_start', default_value='false'),
        DeclareLaunchArgument('stand_when_idle', default_value='false'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('foxglove', default_value='false'),
        OpaqueFunction(function=build_nodes),
    ])
