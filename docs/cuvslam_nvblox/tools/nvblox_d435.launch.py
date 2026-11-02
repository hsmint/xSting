"""nvblox on the front D435 depth stream, posed by cuVSLAM, for the xSting robot.

Needs, all on the Jetson:
  - robot base launch with wheel_odom_tf:=false  (URDF TF, IMU)
  - apply_calib_node + imu_time_shift.py         (/opencr/imu_synced)
  - realsense2_camera with enable_depth:=true    (depth/image_rect_raw)
  - cuvslam_stereo.launch.py imu:=true tf:=true  (map -> odom -> base_footprint)

  ros2 launch ~/calibration/nvblox_d435.launch.py
  ros2 launch ~/calibration/nvblox_d435.launch.py color:=true   # colored mesh
  ros2 launch ~/calibration/nvblox_d435.launch.py voxel_size:=0.05  # coarser map, the base config size

color:=true also needs realsense2_camera with enable_color:=true (color/image_raw). The driver
publishes camera_link -> camera_color_optical_frame, so nvblox projects the color frames onto the
mesh with that TF; the color is only for viewing and does not change the TSDF or the ESDF.

Builds a TSDF map in the odom frame (3 cm voxels by default), a mesh for RViz and a 2D ESDF slice
(distance to the nearest obstacle between 5 and 60 cm above the floor) for Nav2.
Settings start from nvblox_examples_bringup/config/nvblox/nvblox_base.yaml; the overrides below
are for this robot (color off by default, D435 depth up to 3 m, lower rates for the Orin Nano).
"""
import os

import launch
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def is_true(context, name):
    return LaunchConfiguration(name).perform(context).lower() in ('true', '1')


def nvblox(context):
    color = is_true(context, 'color')
    voxel_size = float(LaunchConfiguration('voxel_size').perform(context))
    base_config = os.path.join(get_package_share_directory('nvblox_examples_bringup'),
                               'config', 'nvblox', 'nvblox_base.yaml')
    overrides = {
        'num_cameras': 1,
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
        'max_back_projection_distance': 3.0,
        # The base config decays the TSDF (x0.95 at 5 Hz) and frees decayed blocks, so anything
        # out of view vanished within ~30 s. nvblox rejects 1.0 (must be < 1); 0.9999 at 5 Hz takes
        # about 2 h to drop a surface below the mesh weight, so a scan keeps its map.
        'static_mapper.tsdf_decay_factor': 0.9999,
        # odom z = 0 is the floor. The slice ignores the floor and stops above the robot's top.
        'static_mapper.projective_integrator_max_integration_distance_m': 3.0,  # D435 is noisy beyond ~3 m
        'static_mapper.esdf_slice_height': 0.3,
        'static_mapper.esdf_slice_min_height': 0.05,
        'static_mapper.esdf_slice_max_height': 0.6,
    }
    nvblox_node = ComposableNode(
        name='nvblox_node',
        package='nvblox_ros',
        plugin='nvblox::NvbloxNode',
        parameters=[base_config, overrides],
        remappings=[
            ('camera_0/depth/image', '/camera/camera/depth/image_rect_raw'),
            ('camera_0/depth/camera_info', '/camera/camera/depth/camera_info'),
            ('camera_0/color/image', '/camera/camera/color/image_raw'),
            ('camera_0/color/camera_info', '/camera/camera/color/camera_info'),
        ],
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
        OpaqueFunction(function=nvblox),
    ])
