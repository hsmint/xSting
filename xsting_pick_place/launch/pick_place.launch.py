import math
from pathlib import Path
import tempfile
import xml.etree.ElementTree as ET
import yaml

from ament_index_python.packages import PackageNotFoundError
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import LogInfo
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode


# 패키지 존재 여부 확인
def has_pkg(name):
    try:
        get_package_share_directory(name)
        return True
    except PackageNotFoundError:
        return False


# 대용량 STL(base 128만 폴리곤)의 CPU FCL 충돌 트리를 정밀 기본 도형으로 경량화
def simplify_fcl_collision(robot):
    prim_map = {
        'base': ('cylinder', {'radius': '0.132', 'length': '0.214'},
                 '0.010 0 0.107'),
        'gripper': ('box', {'size': '0.068 0.128 0.104'}, '0.053 0 0.030'),
    }
    for link in robot.findall('link'):
        name = link.get('name')
        if name not in prim_map:
            continue
        for col in list(link.findall('collision')):
            link.remove(col)
        col = ET.SubElement(link, 'collision')
        ET.SubElement(col, 'origin', xyz=prim_map[name][2], rpy='0 0 0')
        geom = ET.SubElement(col, 'geometry')
        ET.SubElement(geom, prim_map[name][0], **prim_map[name][1])


# 실기 하드웨어(xsting_hardware::kArmHome)와 동일한 접힌 시작·홈 자세
FOLDED_HOME = {
    'arm_yaw': 0.0,
    'arm_shoulder': -1.0508,
    'arm_elbow': 1.0707,
    'arm_wrist': 0.0,
    'arm_roll': 0.0,
    'finger_l': 0.0,
    'finger_r': 0.0,
}


# cuMotion CUDA 커널 한계(1024개 구) 이내로 반경 팽창 없이 중복 구를 솎아낸 XRDF 생성
def build_compact_xrdf(src_xrdf):
    data = yaml.safe_load(src_xrdf.read_text(encoding='utf-8'))
    cspace = set(data.get('cspace', {}).get('joint_names', []))
    if 'default_joint_positions' in data and cspace:
        merged = {**data['default_joint_positions'], **FOLDED_HOME}
        data['default_joint_positions'] = {
            k: v for k, v in merged.items() if k in cspace
        }

    spheres = (
        data.get('geometry', {})
        .get('robot_spheres', {})
        .get('spheres', {})
    )
    total = sum(len(v) for v in spheres.values())
    if total > 1000:
        compact = {}
        for link, lst in spheres.items():
            if link == 'base':
                tol = 0.026
            elif link.startswith('leg'):
                tol = 0.045
            else:
                tol = 0.013
            kept = []
            for sp in sorted(lst, key=lambda x: -float(x['radius'])):
                c, r = sp['center'], float(sp['radius'])
                if any(math.dist(c, kc) + r <= kr + tol for kc, kr in kept):
                    continue
                kept.append((c, r))
            compact[link] = [
                {'center': c, 'radius': round(r, 5)} for c, r in kept
            ]
        data['geometry']['robot_spheres']['spheres'] = compact

    out_path = Path(tempfile.gettempdir()) / 'xsting_cumotion_compact.xrdf'
    out_path.write_text(
        yaml.safe_dump(data, sort_keys=False), encoding='utf-8'
    )
    return out_path


# 하드웨어·계획기 옵션에 맞춰 MoveIt + cuMotion + 픽앤플레이스 노드 구성
def build_nodes(context):
    mode = LaunchConfiguration('hardware').perform(context)
    req_pipe = LaunchConfiguration('pipeline').perform(context)
    sim = LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'
    use_rviz = LaunchConfiguration('rviz').perform(context).lower() == 'true'

    mv_share = Path(get_package_share_directory('xsting_moveit_config'))
    mv_cfg = mv_share / 'config'
    pp_share = Path(get_package_share_directory('xsting_pick_place'))
    pp_cfg = pp_share / 'config' / 'pick_place.yaml'
    load_mv = lambda name: yaml.safe_load((mv_cfg / name).read_text())

    # mock 모드는 GenericSystem, isaac 모드는 topic_based 또는 내장 traj_bridge 선택
    robot = ET.parse(mv_cfg / 'xsting_manipulation.urdf').getroot()
    simplify_fcl_collision(robot)
    init_q = {**load_mv('xsting.xrdf')['default_joint_positions'], **FOLDED_HOME}
    use_ros2_ctrl = (
        mode == 'mock' or
        (mode == 'isaac' and has_pkg('topic_based_ros2_control'))
    )
    if use_ros2_ctrl:
        ctrl = ET.SubElement(robot, 'ros2_control', name='xsting_plan',
                             type='system')
        hw = ET.SubElement(ctrl, 'hardware')
        plugin = ('mock_components/GenericSystem' if mode == 'mock'
                  else 'topic_based_ros2_control/TopicBasedSystem')
        ET.SubElement(hw, 'plugin').text = plugin
        if mode == 'isaac':
            ET.SubElement(hw, 'param',
                          name='joint_commands_topic').text = '/isaac_joint_commands'
            ET.SubElement(hw, 'param',
                          name='joint_states_topic').text = '/isaac_joint_states'
        for j_name, q_val in init_q.items():
            joint = ET.SubElement(ctrl, 'joint', name=j_name)
            if j_name != 'finger_r':
                ET.SubElement(joint, 'command_interface', name='position')
            st = ET.SubElement(joint, 'state_interface', name='position')
            ET.SubElement(st, 'param', name='initial_value').text = str(q_val)
            ET.SubElement(joint, 'state_interface', name='velocity')

    ctrl_cfg = load_mv('moveit_controllers.yaml')
    ctrl_cfg.setdefault('trajectory_execution', {}).update({
        'allowed_execution_duration_scaling': 3.5,
        'allowed_goal_duration_margin': 5.0,
        'allowed_start_tolerance': 0.12,
    })
    jlim_cfg = load_mv('joint_limits.yaml')
    for j_name in ('arm_yaw', 'arm_shoulder', 'arm_elbow', 'arm_wrist', 'arm_roll'):
        if j_name in jlim_cfg.get('joint_limits', {}):
            jlim_cfg['joint_limits'][j_name]['max_velocity'] = 2.0
            jlim_cfg['joint_limits'][j_name]['max_acceleration'] = 3.5

    pp_raw = yaml.safe_load(pp_cfg.read_text(encoding='utf-8')) or {}
    pp_vel = float(
        pp_raw.get('/**', {}).get('ros__parameters', {}).get('vel_scale', 0.70)
    )

    desc = {'robot_description': ET.tostring(robot, encoding='unicode')}
    params = {
        **desc,
        'robot_description_semantic': (mv_cfg / 'xsting.srdf').read_text(),
        'robot_description_kinematics': load_mv('kinematics.yaml'),
        'robot_description_planning': jlim_cfg,
        'planning_pipelines': ['ompl'],
        'default_planning_pipeline': 'ompl',
        'ompl': load_mv('ompl_planning.yaml'),
        'use_sim_time': sim,
        'publish_robot_description_semantic': True,
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
        **ctrl_cfg,
    }

    actions = []
    active_pipe = 'ompl'

    # cuMotion 패키지가 설치되어 있으면 GPU 플래너 컨테이너 활성화
    if req_pipe == 'cumotion':
        if has_pkg('isaac_ros_cumotion') and has_pkg('isaac_ros_cumotion_moveit'):
            cm_dir = Path(get_package_share_directory('isaac_ros_cumotion_moveit'))
            cm_yaml = cm_dir / 'config' / 'isaac_ros_cumotion_planning.yaml'
            params['isaac_ros_cumotion'] = yaml.safe_load(cm_yaml.read_text())
            params['planning_pipelines'].append('isaac_ros_cumotion')
            params['default_planning_pipeline'] = 'isaac_ros_cumotion'
            active_pipe = 'isaac_ros_cumotion'

            actions.append(ComposableNodeContainer(
                name='cumotion_container',
                namespace='',
                package='rclcpp_components',
                executable='component_container',
                arguments=['--executor-type', 'multi-threaded'],
                parameters=[{'use_sim_time': sim}],
                output='screen',
                composable_node_descriptions=[
                    ComposableNode(
                        name='static_planning_scene_server',
                        package='isaac_ros_cumotion',
                        plugin='nvidia::isaac_ros::cumotion::StaticPlanningSceneServer',
                        parameters=[{'use_sim_time': sim}],
                    ),
                    ComposableNode(
                        name='cumotion_planner',
                        package='isaac_ros_cumotion',
                        plugin='nvidia::isaac_ros::cumotion::CumotionPlanner',
                        parameters=[{
                            'urdf_file_path': str(mv_cfg / 'xsting_manipulation.urdf'),
                            'xrdf_file_path': str(build_compact_xrdf(mv_cfg / 'xsting.xrdf')),
                            'joint_states_topic': '/joint_states',
                            'use_sim_time': sim,
                            'time_dilation_factor': pp_vel,
                        }],
                    ),
                ],
            ))
        else:
            actions.append(LogInfo(
                msg='[xsting_pick_place] isaac_ros_cumotion 미설치 -> OMPL 폴백 실행'
            ))

    # real 모드는 tool0 정적 TF만 추가하고 그 외 모드는 robot_state_publisher 실행
    if mode == 'real':
        actions.extend([
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='tool0_static_tf',
                arguments=['--x', '0.0995', '--frame-id', 'gripper',
                           '--child-frame-id', 'tool0'],
            ),
            Node(
                package='xsting_pick_place',
                executable='traj_bridge_node',
                name='traj_bridge',
                parameters=[{'use_sim_time': sim}],
                output='screen',
            ),
        ])
    else:
        actions.append(Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[desc, {'use_sim_time': sim}],
            output='screen',
        ))

    # MoveIt move_group 및 컨트롤러 매니저 실행
    actions.append(Node(
        package='moveit_ros_move_group',
        executable='move_group',
        parameters=[params],
        output='screen',
    ))

    if use_ros2_ctrl:
        ctrl_yaml = str(mv_cfg / 'ros2_controllers.yaml')
        actions.extend([
            Node(
                package='controller_manager',
                executable='ros2_control_node',
                parameters=[ctrl_yaml, {'use_sim_time': sim}],
                remappings=[('~/robot_description', '/robot_description')],
                output='screen',
            ),
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=[
                    'joint_state_broadcaster',
                    'arm_trajectory_controller',
                    'gripper_trajectory_controller',
                    '--param-file', ctrl_yaml,
                    '--controller-manager-timeout', '60',
                ],
                output='screen',
            ),
        ])
    elif mode == 'isaac':
        actions.append(Node(
            package='xsting_pick_place',
            executable='traj_bridge_node',
            name='traj_bridge',
            parameters=[{
                'use_sim_time': sim,
                'use_joint_state_cmd': True,
                'cmd_topic': '/isaac_joint_commands',
                'state_topic': '/isaac_joint_states',
            }],
            output='screen',
        ))

    # 픽앤플레이스 액션 노드 및 선택적 RViz 실행
    actions.append(Node(
        package='xsting_pick_place',
        executable='pick_place_node',
        parameters=[
            str(pp_cfg),
            {
                **desc,
                'robot_description_semantic': (mv_cfg / 'xsting.srdf').read_text(),
                'pipeline': active_pipe,
                'use_sim_time': sim,
                'check_grasp': mode != 'mock',
                'robot_description_kinematics': load_mv('kinematics.yaml'),
            },
        ],
        output='screen',
    ))

    if use_rviz:
        actions.append(Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', str(mv_cfg / 'moveit.rviz')],
            parameters=[params],
        ))
    return actions


# 런치 인자 선언 및 엔트리 반환
def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('hardware', default_value='mock',
                              choices=['mock', 'isaac', 'real', 'external']),
        DeclareLaunchArgument('pipeline', default_value='cumotion',
                              choices=['cumotion', 'ompl']),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='false'),
        OpaqueFunction(function=build_nodes),
    ])
