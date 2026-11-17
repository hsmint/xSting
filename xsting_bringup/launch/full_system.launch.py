"""전체 로봇 시스템 마스터 통합 런치 파일 (하드웨어 + 비전/SLAM + RL 주행/정밀접근 + 픽앤플레이스 + 미션)."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _pkg_file(pkg, rel_path):
    return os.path.join(get_package_share_directory(pkg), rel_path)


def _is_true(context, name):
    val = LaunchConfiguration(name).perform(context).strip().lower()
    return val in ('true', '1', 'yes', 'on')


def _setup_full_system(context):
    with_hardware = _is_true(context, 'with_hardware')
    with_vision = _is_true(context, 'with_vision')
    with_nav = _is_true(context, 'with_nav')
    with_approach = _is_true(context, 'with_approach')
    with_pick_place = _is_true(context, 'with_pick_place')
    with_mission = _is_true(context, 'with_mission')

    dry_run = LaunchConfiguration('dry_run').perform(context).strip().lower()
    foxglove = LaunchConfiguration('foxglove').perform(context).strip().lower()
    rviz = LaunchConfiguration('rviz').perform(context).strip().lower()
    lidar_port = LaunchConfiguration('lidar_port').perform(context).strip()
    pipeline = LaunchConfiguration('pipeline').perform(context).strip()
    commander = LaunchConfiguration('commander').perform(context).strip()
    decider = LaunchConfiguration('decider').perform(context).strip()
    guard = LaunchConfiguration('guard').perform(context).strip()
    record_media = LaunchConfiguration('record_media').perform(context).strip()
    model = LaunchConfiguration('model').perform(context).strip()
    effort = LaunchConfiguration('effort').perform(context).strip()
    nav_mode_cfg = LaunchConfiguration('nav_mode').perform(context).strip()
    nav_mode = ('action' if with_nav else 'manual') if nav_mode_cfg == 'auto' else nav_mode_cfg

    actions = []

    # 1. 실기 하드웨어 제어기 (robot_state_publisher + ros2_control + joint_state_broadcaster 단일 소유)
    if with_hardware:
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    _pkg_file('xsting_hardware', 'launch/hardware.launch.py')
                ),
                launch_arguments={
                    'use_leg': LaunchConfiguration('use_leg').perform(context),
                    'use_arm': LaunchConfiguration('use_arm').perform(context),
                    'stand': LaunchConfiguration('stand').perform(context),
                    'reboot': LaunchConfiguration('reboot').perform(context),
                }.items(),
            )
        )

    # 2. 비전·cuVSLAM·nvblox·YOLOv8·신호등/바코드 컨테이너
    #    하드웨어 또는 mock 픽앤플레이스가 켜져 있으면 중복 robot_state_publisher / joint_state_publisher 차단
    if with_vision:
        ext_state_owner = with_hardware or with_pick_place
        imu_arg = LaunchConfiguration('imu').perform(context).strip().lower()
        if imu_arg == 'auto':
            imu_enabled = 'true' if with_hardware else 'false'
        else:
            imu_enabled = imu_arg
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    _pkg_file('xsting_bringup', 'launch/xsting.launch.py')
                ),
                launch_arguments={
                    'hardware': 'external' if ext_state_owner else 'false',
                    'imu': imu_enabled,
                    'imu_topic': '/imu_broadcaster/imu',
                    'lidar_port': lidar_port,
                    'foxglove': foxglove,
                    'rviz': rviz,
                }.items(),
            )
        )

    # 3. 41D RL 보행 주행(/nav/run) + 상자 2cm 정밀 접근(/approach_box)
    if with_nav:
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    _pkg_file('xsting_nav', 'launch/nav.launch.py')
                ),
                launch_arguments={
                    'with_approach': 'true' if with_approach else 'false',
                    'dry_run': dry_run,
                    'calib_file': LaunchConfiguration('calib_file').perform(context),
                }.items(),
            )
        )

    # 4. MoveIt + cuMotion + ArduCam 탑뷰 픽앤플레이스(/pick_place)
    if with_pick_place:
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    _pkg_file('xsting_pick_place', 'launch/pick_place.launch.py')
                ),
                launch_arguments={
                    'hardware': 'real' if with_hardware else 'mock',
                    'pipeline': pipeline,
                    'rviz': 'false',
                }.items(),
            )
        )

    # 5. 상위 미션 오케스트레이션 (mission_bridge + robot_command_server + 선택적 llm_commander)
    if with_mission:
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    _pkg_file('xsting_mission', 'launch/mission.launch.py')
                ),
                launch_arguments={
                    'backend': LaunchConfiguration('backend').perform(context),
                    'nav_mode': nav_mode,
                    'commander': commander,
                    'decider': decider,
                    'guard': guard,
                    'record_media': record_media,
                    'model': model,
                    'effort': effort,
                }.items(),
            )
        )

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('with_hardware', default_value='true'),
        DeclareLaunchArgument('with_vision', default_value='true'),
        DeclareLaunchArgument('with_nav', default_value='true'),
        DeclareLaunchArgument('with_approach', default_value='true'),
        DeclareLaunchArgument('with_pick_place', default_value='true'),
        DeclareLaunchArgument('with_mission', default_value='true'),
        DeclareLaunchArgument('use_leg', default_value='true'),
        DeclareLaunchArgument('use_arm', default_value='true'),
        DeclareLaunchArgument('stand', default_value='true'),
        DeclareLaunchArgument('reboot', default_value='false'),
        DeclareLaunchArgument('imu', default_value='auto'),
        DeclareLaunchArgument('lidar_port', default_value='/dev/lidar'),
        DeclareLaunchArgument('calib_file', default_value=''),
        DeclareLaunchArgument('dry_run', default_value='false'),
        DeclareLaunchArgument('pipeline', default_value='cumotion'),
        DeclareLaunchArgument('backend', default_value='ros'),
        DeclareLaunchArgument('nav_mode', default_value='auto'),
        DeclareLaunchArgument('commander', default_value='false'),
        DeclareLaunchArgument('decider', default_value='llm'),
        DeclareLaunchArgument('guard', default_value='true'),
        DeclareLaunchArgument('record_media', default_value='true'),
        DeclareLaunchArgument('model', default_value=''),
        DeclareLaunchArgument('effort', default_value=''),
        DeclareLaunchArgument('foxglove', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='false'),
        OpaqueFunction(function=_setup_full_system),
    ])
