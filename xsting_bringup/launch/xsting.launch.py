import os
import platform

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode
import yaml

# 공유 멀티스레드 컴포넌트 컨테이너 이름 및 제로카피 통신 옵션
CONTAINER_NAME = 'vision'
INTRA_ARGS = [{'use_intra_process_comms': True}]

# 비압축 영상·내부 텐서·대용량 맵은 제외하고 압축 영상·Action·필수 토픽만 허용
FG_WHITELIST = [
    r'^(?!'
    r'.*/(?!_action/(feedback|status)$)[^A-Za-z~].*$'
    r'|.*/yolo/(encoder|tensor_pub|tensor_sub|output_tensor)(/.*)?$'
    r'|/sign_yolo/(tensor_.*|output_tensor)$'
    r'|/camera/(infra1|infra2)/.*$'
    r'|/camera/(color|depth)/image_(raw|rect_raw|emitter_.*)$'
    r'|/camera/depth/metadata$'
    r'|/arducam/image_rect$'
    r'|/nvblox_node/(mesh|.*_layer|.*_map_slice|back_projected_depth/.*)$'
    r'|/nvblox_node/(tsdf|freespace|dynamic_occupancy)_layer_marker$'
    r'|/visual_slam/(status|tracking/vo_odometry)$'
    r'|/lidar/pointcloud$'
    r').*$'
]


# 패키지 공유 디렉토리 기준 파일 절대 경로 반환
def get_pkg_path(pkg, rel_path):
    return os.path.join(get_package_share_directory(pkg), rel_path)


# 현재 CPU 아키텍처에 맞는 모델 하위 폴더 이름 반환
def get_model_arch():
    arch = platform.machine().lower()
    if arch in ('aarch64', 'arm64'):
        return 'arm64'
    if arch in ('x86_64', 'amd64'):
        return 'x86_64'
    raise RuntimeError(f'Unsupported architecture: {arch}')


# 런치 문자열 인자를 불리언 값으로 변환
def is_enabled(context, name):
    val = LaunchConfiguration(name).perform(context).strip().lower()
    return val in ('true', '1', 'yes', 'on')


# 카메라 한 대의 경량 GPU 전처리 인코더 컴포넌트 목록 생성
def create_yolo_encoder(cam):
    ns = f'{cam}/yolo'
    enc_ns = f'{ns}/encoder'
    return [
        ComposableNode(
            name='image_to_tensor',
            namespace=enc_ns,
            package='isaac_ros_tensor_proc',
            plugin='nvidia::isaac_ros::dnn_inference::ImageToTensorNode',
            parameters=[{
                'scale': True,
                'tensor_name': 'image',
                'input_qos': 'SENSOR_DATA',
            }],
            remappings=[
                ('image', f'/{cam}/color/image_raw'),
                ('tensor', 'image_tensor'),
            ],
            extra_arguments=INTRA_ARGS,
        ),
        ComposableNode(
            name='interleaved_to_planar_node',
            namespace=enc_ns,
            package='isaac_ros_tensor_proc',
            plugin='nvidia::isaac_ros::dnn_inference::InterleavedToPlanarNode',
            parameters=[{
                'input_tensor_shape': [480, 640, 3],
            }],
            remappings=[
                ('interleaved_tensor', 'image_tensor'),
                ('planar_tensor', 'planar_tensor'),
            ],
            extra_arguments=INTRA_ARGS,
        ),
        ComposableNode(
            name='reshape_node',
            namespace=enc_ns,
            package='isaac_ros_tensor_proc',
            plugin='nvidia::isaac_ros::dnn_inference::ReshapeNode',
            parameters=[{
                'output_tensor_name': 'input_tensor',
                'input_tensor_layout': 'CHW',
                'output_tensor_layout': 'NCHW',
                'input_tensor_shape': [3, 480, 640],
                'output_tensor_shape': [1, 3, 480, 640],
            }],
            remappings=[
                ('tensor', 'planar_tensor'),
                ('reshaped_tensor', f'/{ns}/tensor_pub'),
            ],
            extra_arguments=INTRA_ARGS,
        ),
    ]


# 실행 옵션에 맞춰 단일 컨테이너 기반 노드 구성을 생성
def setup_launch(context):
    hw_mode = LaunchConfiguration('hardware').perform(context).strip().lower()
    hw_internal = hw_mode in ('true', '1', 'yes', 'on', 'real')
    hw_active = hw_internal or hw_mode == 'external'

    raw_urdf = is_enabled(context, 'urdf')
    # 실기 하드웨어(또는 외부 컨트롤러)가 켜진 경우 중복 robot_state_publisher·joint_state_publisher 차단
    use_urdf = raw_urdf and not hw_active
    use_joint_states = is_enabled(context, 'joint_states') and not hw_active
    use_realsense = is_enabled(context, 'realsense')
    use_arducam = is_enabled(context, 'arducam')
    use_lidar = is_enabled(context, 'lidar')
    use_slam = is_enabled(context, 'slam')
    use_imu = is_enabled(context, 'imu')
    use_map_3d = is_enabled(context, 'map_3d')
    use_yolo = is_enabled(context, 'yolo')
    use_sign = is_enabled(context, 'sign')
    use_foxglove = is_enabled(context, 'foxglove')
    use_rviz = is_enabled(context, 'rviz')
    lidar_port = LaunchConfiguration('lidar_port').perform(context).strip()
    imu_topic = LaunchConfiguration('imu_topic').perform(context).strip()
    if hw_active and imu_topic == '/imu':
        imu_topic = '/imu_broadcaster/imu'

    actions = []
    nodes = []

    # hardware:=true 지정 시 xsting_hardware 런치를 충돌 없이 내부에서 함께 기동
    if hw_internal:
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    get_pkg_path('xsting_hardware', 'launch/hardware.launch.py')
                ),
                launch_arguments={
                    'use_leg': LaunchConfiguration('use_leg').perform(context),
                    'use_arm': LaunchConfiguration('use_arm').perform(context),
                    'stand': LaunchConfiguration('stand').perform(context),
                    'reboot': LaunchConfiguration('reboot').perform(context),
                }.items(),
            )
        )

    # URDF 기반 로봇 정적·관절 TF 발행 노드 실행
    if use_urdf:
        urdf_path = get_pkg_path('xsting_description', 'urdf/xsting.urdf')
        with open(urdf_path, 'r', encoding='utf-8') as urdf_file:
            urdf_xml = urdf_file.read()
        actions.append(
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                parameters=[{'robot_description': urdf_xml}],
            )
        )
        # 외부 하드웨어 joint_states 없이도 다리·팔·arducam 기본 서기 자세 TF 완성
        if use_joint_states:
            order_path = get_pkg_path(
                'xsting_description', 'config/joint_order.yaml'
            )
            with open(order_path, 'r', encoding='utf-8') as order_file:
                order = yaml.safe_load(order_file)
            zeros = {
                j['name']: float(j['default'])
                for val in order.values() if isinstance(val, list)
                for j in val if isinstance(j, dict) and 'default' in j
            }
            actions.append(
                Node(
                    package='joint_state_publisher',
                    executable='joint_state_publisher',
                    name='joint_state_publisher',
                    parameters=[{'zeros': zeros}] if zeros else [],
                )
            )

    # 전면 RealSense 카메라 드라이버 및 에미터 분리 노드 추가
    if use_realsense:
        nodes.extend([
            ComposableNode(
                name='realsense_camera',
                package='xsting_vision',
                plugin='xsting_vision::RealsenseCameraNode',
                extra_arguments=INTRA_ARGS,
            ),
            ComposableNode(
                name='realsense_splitter',
                package='xsting_vision',
                plugin='xsting_vision::RealsenseSplitterNode',
                extra_arguments=INTRA_ARGS,
            ),
        ])

    # Arducam 로봇팔 카메라 GPU 디코딩·왜곡 보정 노드 추가
    if use_arducam:
        nodes.append(
            ComposableNode(
                name='arducam_camera',
                package='xsting_vision',
                plugin='xsting_vision::ArducamCameraNode',
                extra_arguments=INTRA_ARGS,
            )
        )

    # 1층 중앙 2D LiDAR 드라이버(독립 시리얼 프로세스) 및 다리 마스킹·ICP 보정 컴포넌트 추가
    if use_lidar:
        coin_d4_cfg = get_pkg_path('xsting_bringup', 'config/coin_d4.yaml')
        actions.append(
            Node(
                package='coin_d4_driver',
                executable='single_coin_d4_node',
                name='coin_d4_lidar',
                parameters=[coin_d4_cfg, {'port': lidar_port}],
                output='screen',
                sigterm_timeout='15',
                sigkill_timeout='15',
            )
        )
        nodes.append(
            ComposableNode(
                name='lidar_odom',
                package='xsting_vision',
                plugin='xsting_vision::LidarOdomNode',
                parameters=[{
                    'enable_odom_fusion': use_slam,
                    'publish_odom_tf': use_slam,
                    'publish_pointcloud': use_slam,
                }],
                extra_arguments=INTRA_ARGS,
            )
        )

    # 전면 camera 기반 cuVSLAM 및 nvblox(카메라 + 2D LiDAR 장애물 매핑) 노드 추가
    if use_slam:
        cuvslam_cfg = get_pkg_path('xsting_bringup', 'config/cuvslam.yaml')
        nvblox_cfg = get_pkg_path('xsting_bringup', 'config/nvblox.yaml')
        nvblox_params = [nvblox_cfg]
        if use_lidar:
            nvblox_params.append({'use_lidar': True})
        if not use_map_3d:
            nvblox_params.append({
                'update_mesh_rate_hz': 0.0,
                'publish_layer_rate_hz': 0.0,
                'static_mapper.workspace_bounds_max_height_m': 0.6,
            })

        nvblox_remappings = [
            ('camera_0/depth/image', '/camera/depth/image_emitter_on'),
            ('camera_0/depth/camera_info', '/camera/depth/camera_info'),
            ('camera_0/color/image', '/camera/color/image_raw'),
            ('camera_0/color/camera_info', '/camera/color/camera_info'),
        ]
        if use_lidar:
            nvblox_remappings.append(('pointcloud', '/lidar/pointcloud'))

        cuvslam_remappings = [
            (
                'visual_slam/image_0',
                '/camera/infra1/image_emitter_off',
            ),
            (
                'visual_slam/camera_info_0',
                '/camera/infra1/camera_info',
            ),
            (
                'visual_slam/image_1',
                '/camera/infra2/image_emitter_off',
            ),
            (
                'visual_slam/camera_info_1',
                '/camera/infra2/camera_info',
            ),
            ('visual_slam/imu', imu_topic),
        ]
        if use_lidar:
            cuvslam_remappings.append((
                'visual_slam/tracking/odometry',
                '/visual_slam/tracking/vo_odometry',
            ))

        nodes.extend([
            ComposableNode(
                name='visual_slam_node',
                package='isaac_ros_cuvslam',
                plugin='nvidia::isaac_ros::visual_slam::VisualSlamNode',
                parameters=[
                    cuvslam_cfg,
                    {
                        'tracking_mode': 1 if use_imu else 0,
                        'publish_odom_to_base_tf': not use_lidar,
                    },
                ],
                remappings=cuvslam_remappings,
                extra_arguments=INTRA_ARGS,
            ),
            ComposableNode(
                name='nvblox_node',
                package='nvblox_ros',
                plugin='nvblox::NvbloxNode',
                parameters=nvblox_params,
                remappings=nvblox_remappings,
                extra_arguments=INTRA_ARGS,
            ),
        ])
    elif raw_urdf:
        # SLAM 비활성화 시에도 RViz odom 기준 좌표계가 유지되도록 정적 TF 발행
        actions.append(
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='static_odom_tf',
                arguments=[
                    '--frame-id',
                    'odom',
                    '--child-frame-id',
                    'base',
                ],
            )
        )

    # 전면 camera의 TensorRT 추론·Isaac ROS 디코더 및 3D PCA 노드 추가
    arch = get_model_arch()
    if use_yolo:
        model_path = get_pkg_path(
            'xsting_vision', 'model/onnx/rg_nano_batch1_640_fp16.onnx'
        )
        engine_path = get_pkg_path(
            'xsting_vision', f'model/{arch}/rg_nano_batch1_640_fp16.engine'
        )
        ns = 'camera/yolo'
        nodes.extend([
            ComposableNode(
                name='tensor_rt',
                namespace=ns,
                package='isaac_ros_tensor_rt',
                plugin='nvidia::isaac_ros::dnn_inference::TensorRTNode',
                parameters=[{
                    'model_file_path': model_path,
                    'engine_file_path': engine_path,
                    'force_engine_update': False,
                    'input_tensor_names': ['input_tensor'],
                    'input_binding_names': ['images'],
                    'output_tensor_names': ['output_tensor'],
                    'output_binding_names': ['output0'],
                }],
                extra_arguments=INTRA_ARGS,
            ),
            ComposableNode(
                name='yolov8_decoder',
                namespace=ns,
                package='isaac_ros_yolov8',
                plugin='nvidia::isaac_ros::yolov8::YoloV8DecoderNode',
                parameters=[{
                    'confidence_threshold': 0.5,
                    'nms_threshold': 0.45,
                    'num_classes': 5,
                }],
                remappings=[
                    ('detections_output', '/camera/color/detections'),
                ],
                extra_arguments=INTRA_ARGS,
            ),
            ComposableNode(
                name='depth_pca',
                package='xsting_vision',
                plugin='xsting_vision::DepthPcaNode',
                extra_arguments=INTRA_ARGS,
            ),
        ])

    # 신호등 전용 TensorRT 추론·Isaac ROS 디코더 및 바코드 Action 노드 추가
    if use_sign:
        sign_model = get_pkg_path(
            'xsting_vision', 'model/onnx/sign_nano_batch1_640_fp16.onnx'
        )
        sign_engine = get_pkg_path(
            'xsting_vision', f'model/{arch}/sign_nano_batch1_640_fp16.engine'
        )
        nodes.extend([
            ComposableNode(
                name='tensor_rt',
                namespace='sign_yolo',
                package='isaac_ros_tensor_rt',
                plugin='nvidia::isaac_ros::dnn_inference::TensorRTNode',
                parameters=[{
                    'model_file_path': sign_model,
                    'engine_file_path': sign_engine,
                    'force_engine_update': False,
                    'input_tensor_names': ['input_tensor'],
                    'input_binding_names': ['images'],
                    'output_tensor_names': ['output_tensor'],
                    'output_binding_names': ['output0'],
                }],
                remappings=[
                    ('tensor_pub', '/camera/yolo/tensor_pub'),
                ],
                extra_arguments=INTRA_ARGS,
            ),
            ComposableNode(
                name='yolov8_decoder',
                namespace='sign_yolo',
                package='isaac_ros_yolov8',
                plugin='nvidia::isaac_ros::yolov8::YoloV8DecoderNode',
                parameters=[{
                    'confidence_threshold': 0.5,
                    'nms_threshold': 0.45,
                    'num_classes': 4,
                }],
                remappings=[
                    ('detections_output', '/sign_yolo/detections'),
                ],
                extra_arguments=INTRA_ARGS,
            ),
            ComposableNode(
                name='sign_barcode',
                package='xsting_vision',
                plugin='xsting_vision::SignBarcodeNode',
                extra_arguments=INTRA_ARGS,
            ),
        ])

    # 전면 카메라의 경량 GPU 전처리 인코더 컴포넌트 추가
    if use_yolo or use_sign:
        nodes.extend(create_yolo_encoder('camera'))

    # 모든 GPU 컴포넌트를 단일 멀티스레드 컨테이너에 적재
    if nodes:
        actions.append(
            ComposableNodeContainer(
                name=CONTAINER_NAME,
                namespace='',
                package='rclcpp_components',
                executable='component_container',
                arguments=['--executor-type', 'multi-threaded'],
                composable_node_descriptions=nodes,
                output='screen',
            )
        )

    # 비전 컨테이너와 격리된 독립 프로세스로 Foxglove 웹소켓 브리지 실행
    if use_foxglove:
        actions.append(
            Node(
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
                    'capabilities': [
                        'clientPublish',
                        'services',
                        'connectionGraph',
                        'assets',
                    ],
                    'topic_whitelist': FG_WHITELIST,
                }],
                output='screen',
            )
        )

    # RViz2 시각화 실행
    if use_rviz:
        rviz_cfg = get_pkg_path('xsting_bringup', 'config/xsting.rviz')
        actions.append(
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', rviz_cfg],
                output='screen',
            )
        )

    return actions


# 명령줄 인자 선언 및 통합 런치 설명 반환
def generate_launch_description():
    args = [
        DeclareLaunchArgument('hardware', default_value='false'),
        DeclareLaunchArgument('use_leg', default_value='true'),
        DeclareLaunchArgument('use_arm', default_value='true'),
        DeclareLaunchArgument('stand', default_value='true'),
        DeclareLaunchArgument('reboot', default_value='false'),
        DeclareLaunchArgument('urdf', default_value='true'),
        DeclareLaunchArgument('joint_states', default_value='true'),
        DeclareLaunchArgument('realsense', default_value='true'),
        DeclareLaunchArgument('arducam', default_value='true'),
        DeclareLaunchArgument('lidar', default_value='true'),
        DeclareLaunchArgument('lidar_port', default_value='/dev/lidar'),
        DeclareLaunchArgument('slam', default_value='true'),
        DeclareLaunchArgument('imu', default_value='false'),
        DeclareLaunchArgument('imu_topic', default_value='/imu'),
        DeclareLaunchArgument('map_3d', default_value='true'),
        DeclareLaunchArgument('yolo', default_value='true'),
        DeclareLaunchArgument('sign', default_value='true'),
        DeclareLaunchArgument('foxglove', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='false'),
    ]
    return LaunchDescription(args + [OpaqueFunction(function=setup_launch)])
