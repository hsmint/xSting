from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def default_calib_path(share: Path) -> str:
    for root in [Path.cwd(), *share.resolve().parents]:
        cand = root / 'src' / 'xsting_nav' / 'config' / 'calibration.yaml'
        if cand.exists():
            return str(cand)
    return str(share / 'config' / 'calibration.yaml')


def build_nodes(context):
    share = Path(get_package_share_directory('xsting_nav'))
    cfg_file = LaunchConfiguration('config_file').perform(context)
    calib_file = LaunchConfiguration('calib_file').perform(context)
    goal_id = int(LaunchConfiguration('goal_id').perform(context))
    pre_tol = float(LaunchConfiguration('pre_approach_tol_m').perform(context))
    dry_run = LaunchConfiguration('dry_run').perform(context).lower() == 'true'
    with_approach = (
        LaunchConfiguration('with_approach').perform(context).lower() == 'true'
    )
    sim = LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'

    overrides = {
        'calib_file': calib_file or default_calib_path(share),
        'goal_id': goal_id,
        'pre_approach_tol_m': pre_tol,
        'dry_run': dry_run,
        'use_sim_time': sim,
    }

    actions = [
        Node(
            package='xsting_nav',
            executable='nav_node',
            name='nav_node',
            output='screen',
            parameters=[
                cfg_file or str(share / 'config' / 'nav.yaml'),
                overrides,
            ],
        )
    ]

    if with_approach:
        app_share = Path(get_package_share_directory('xsting_approach'))
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(app_share / 'launch' / 'approach.launch.py')
                ),
                launch_arguments={
                    'dry_run': 'true' if dry_run else 'false',
                    'stand_when_idle': 'false',
                    'use_sim_time': 'true' if sim else 'false',
                }.items(),
            )
        )

    return actions


def generate_launch_description():
    share = Path(get_package_share_directory('xsting_nav'))
    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file', default_value=str(share / 'config' / 'nav.yaml')
        ),
        DeclareLaunchArgument('calib_file', default_value=''),
        DeclareLaunchArgument('goal_id', default_value='-1'),
        DeclareLaunchArgument('pre_approach_tol_m', default_value='0.08'),
        DeclareLaunchArgument('dry_run', default_value='false'),
        DeclareLaunchArgument('with_approach', default_value='false'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        OpaqueFunction(function=build_nodes),
    ])
