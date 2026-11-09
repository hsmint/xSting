"""nvblox on the D435 depth streams (front, optionally the 3rd-deck one), posed by cuVSLAM, for the xSting robot.

Needs, all on the Jetson:
  - robot base launch with wheel_odom_tf:=false  (URDF TF, IMU)
  - apply_calib_node + imu_time_shift.py         (/opencr/imu_synced)
  - realsense2_camera with enable_depth:=true    (depth/image_rect_raw)
  - cuvslam_stereo.launch.py imu:=true tf:=true  (map -> odom -> base_footprint)

  ros2 launch ~/calibration/nvblox_d435.launch.py
  ros2 launch ~/calibration/nvblox_d435.launch.py color:=true   # colored mesh
  ros2 launch ~/calibration/nvblox_d435.launch.py voxel_size:=0.05  # coarser map, the base config size
  ros2 launch ~/calibration/nvblox_d435.launch.py max_distance:=4.0  # integrate depth out to 4 m
  ros2 launch ~/calibration/nvblox_d435.launch.py lower:=true       # + 3rd-deck D435 (camera_name:=lower_camera)

Replaying a bag from record_nvblox_bag.sh (laptop, no robot): the bag carries the raw sensors, so cuVSLAM
runs on the laptop too and gives the poses (a bag recorded with --with-slam already has them: skip cuVSLAM).
Start cuVSLAM and nvblox, then the bag with --clock:
  ros2 launch docs/cuvslam_nvblox/tools/cuvslam_stereo.launch.py imu:=true tf:=true use_sim_time:=true
  ros2 launch docs/cuvslam_nvblox/tools/nvblox_d435.launch.py use_sim_time:=true color:=true
  ros2 bag play <bag> --clock
Every argument here only changes how nvblox processes the depth, so the same bag can be replayed with other
values (max_distance, voxel_size, color, lower if the bag has the 3rd-deck camera).

color:=true also needs realsense2_camera with enable_color:=true (color/image_raw). The driver
publishes camera_link -> camera_color_optical_frame, so nvblox projects the color frames onto the
mesh with that TF; the color is only for viewing and does not change the TSDF or the ESDF.

Builds a TSDF map in the odom frame (3 cm voxels by default), a mesh for RViz and a 2D ESDF slice
(distance to the nearest obstacle between 5 and 60 cm above the floor) for Nav2.
Settings start from nvblox_base.yaml next to this file (a copy of nvblox_examples_bringup's); the overrides below
are for this robot (color off by default, D435 depth up to max_distance, lower rates for the Orin Nano).
"""
import os

import launch
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def is_true(context, name):
    return LaunchConfiguration(name).perform(context).lower() in ('true', '1')


def nvblox(context):
    color = is_true(context, 'color')
    # Bag replay: follow /clock, so TF lookups at the image stamps and the rate timers use bag time.
    use_sim_time = is_true(context, 'use_sim_time')
    voxel_size = float(LaunchConfiguration('voxel_size').perform(context))
    # D435 depth error grows with the square of the range: a few cm at 3 m, 10+ cm at 4-5 m (more with the
    # projector off), which shows up as smeared walls and streaks. Integration and back projection share it.
    max_distance = float(LaunchConfiguration('max_distance').perform(context))
    # The 3rd-deck D435 adds a second depth input; its pose comes from the URDF (lower_d435_joint, CAD).
    lower = is_true(context, 'lower')
    # Local copy, so only nvblox_ros has to be installed (nvblox_examples_bringup pulls in Triton and demo models).
    base_config = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'nvblox_base.yaml')
    overrides = {
        'use_sim_time': use_sim_time,
        'num_cameras': 2 if lower else 1,
        # The base config uses 5 cm. Smaller voxels show more detail but grow the TSDF ~(5/size)^3 and
        # the mesh sent to RViz ~(5/size)^2; the truncation distance is in voxels, so it follows.
        'voxel_size': voxel_size,
        'use_depth': True,
        'use_color': color,
        # Color only paints the mesh, so 5 Hz (the base config) is enough and keeps the Orin Nano free.
        'integrate_color_rate_hz': 5.0,
        'use_lidar': False,
        'global_frame': 'odom',
        'input_qos': 'SENSOR_DATA',
        'map_clearing_frame_id': 'base_footprint',
        'esdf_slice_bounds_visualization_attachment_frame_id': 'base_footprint',
        'workspace_height_bounds_visualization_attachment_frame_id': 'base_footprint',
        # Orin Nano: integrate half the 30 fps depth frames. The mesh goes to RViz over Wi-Fi and the map
        # keeps growing (no decay), so it is rebuilt and sent only every 5 s. publish_layer_rate_hz is what
        # sends it: at 2 Hz with 3 cm voxels each message was ~5.5 MB (~10 MB/s), which filled the Wi-Fi and
        # froze RViz. The streamer limit caps what is left; these rates are read only at startup.
        'integrate_depth_rate_hz': 15.0,
        'update_mesh_rate_hz': 0.2,
        'publish_layer_rate_hz': 0.2,
        'layer_streamer_bandwidth_limit_mbps': 10.0,
        # The ESDF point cloud and occupancy grid are resent whole on every ESDF update and grow with the
        # map: at 10 Hz the point cloud alone was ~3 MB/s to RViz. 2 Hz is still enough for a Nav2 costmap.
        'update_esdf_rate_hz': 2.0,
        'max_back_projection_distance': max_distance,
        # The base config decays the TSDF (x0.95 at 5 Hz) and frees decayed blocks, so anything
        # out of view vanished within ~30 s. nvblox rejects 1.0 (must be < 1); 0.9999 at 5 Hz takes
        # about 2 h to drop a surface below the mesh weight, so a scan keeps its map.
        'static_mapper.tsdf_decay_factor': 0.9999,
        # odom z = 0 is the floor. The slice ignores the floor and stops above the robot's top.
        'static_mapper.projective_integrator_max_integration_distance_m': max_distance,
        'static_mapper.esdf_slice_height': 0.3,
        'static_mapper.esdf_slice_min_height': 0.05,
        'static_mapper.esdf_slice_max_height': 0.6,
    }
    # camera_N inputs of nvblox_node -> RealSense driver namespaces (camera_namespace:=camera, camera_name:=...).
    cameras = ['/camera/camera'] + (['/camera/lower_camera'] if lower else [])
    remappings = []
    for i, ns in enumerate(cameras):
        remappings += [
            (f'camera_{i}/depth/image', f'{ns}/depth/image_rect_raw'),
            (f'camera_{i}/depth/camera_info', f'{ns}/depth/camera_info'),
            (f'camera_{i}/color/image', f'{ns}/color/image_raw'),
            (f'camera_{i}/color/camera_info', f'{ns}/color/camera_info'),
        ]
    nvblox_node = ComposableNode(
        name='nvblox_node',
        package='nvblox_ros',
        plugin='nvblox::NvbloxNode',
        parameters=[base_config, overrides],
        remappings=remappings,
    )
    container = ComposableNodeContainer(
        name='nvblox_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[nvblox_node],
        output='screen',
    )
    return [container]


def generate_launch_description():
    return launch.LaunchDescription([
        DeclareLaunchArgument('color', default_value='false',
                              description='color the mesh from the D435 RGB stream '
                                          '(realsense2_camera with enable_color:=true)'),
        DeclareLaunchArgument('voxel_size', default_value='0.03',
                              description='TSDF voxel edge in meters (base config 0.05)'),
        DeclareLaunchArgument('max_distance', default_value='3.0',
                              description='farthest depth in meters that goes into the map'),
        DeclareLaunchArgument('lower', default_value='false',
                              description='also use the 3rd-deck D435 (realsense2_camera camera_name:=lower_camera)'),
        DeclareLaunchArgument('use_sim_time', default_value='false',
                              description='true when replaying a bag with ros2 bag play --clock'),
        OpaqueFunction(function=nvblox),
    ])
