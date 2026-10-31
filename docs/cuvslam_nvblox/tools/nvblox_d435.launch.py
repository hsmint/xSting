"""nvblox on the front D435 depth stream, posed by cuVSLAM, for the xSting robot.

Needs, all on the Jetson:
  - robot base launch with wheel_odom_tf:=false  (URDF TF, IMU)
  - apply_calib_node + imu_time_shift.py         (/opencr/imu_synced)
  - realsense2_camera with enable_depth:=true    (depth/image_rect_raw)
  - cuvslam_stereo.launch.py imu:=true tf:=true  (map -> odom -> base_footprint)

  ros2 launch ~/calibration/nvblox_d435.launch.py

Builds a TSDF map in the odom frame (5 cm voxels), a mesh for RViz and a 2D ESDF slice
(distance to the nearest obstacle between 5 and 60 cm above the floor) for Nav2.
Settings start from nvblox_examples_bringup/config/nvblox/nvblox_base.yaml; the overrides below
are for this robot (no color, D435 depth up to 3 m, lower rates for the Orin Nano).
"""
import os

import launch
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    base_config = os.path.join(get_package_share_directory('nvblox_examples_bringup'),
                               'config', 'nvblox', 'nvblox_base.yaml')
    overrides = {
        'num_cameras': 1,
        'use_depth': True,
        'use_color': False,
        'use_lidar': False,
        'global_frame': 'odom',
        'input_qos': 'SENSOR_DATA',
        'map_clearing_frame_id': 'base_footprint',
        'esdf_slice_bounds_visualization_attachment_frame_id': 'base_footprint',
        'workspace_height_bounds_visualization_attachment_frame_id': 'base_footprint',
        # Orin Nano: integrate half the 30 fps depth frames, refresh the mesh twice a second.
        'integrate_depth_rate_hz': 15.0,
        'update_mesh_rate_hz': 2.0,
        'publish_layer_rate_hz': 2.0,
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
    return launch.LaunchDescription([container])
