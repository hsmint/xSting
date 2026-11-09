"""cuVSLAM on the D435 IR pair of the xSting robot, stereo only or with the OpenCR IMU.

Start the RealSense driver first (camera_namespace:=camera camera_name:=camera, infra1/2 on,
emitter off).

  ros2 launch ~/calibration/cuvslam_stereo.launch.py                  # stereo, robot base launch may run too
  ros2 launch ~/calibration/cuvslam_stereo.launch.py viz:=true        # stereo + TF/clouds for RViz, robot base launch must be off
  ros2 launch ~/calibration/cuvslam_stereo.launch.py imu:=true        # stereo + IMU, robot base launch must run
  ros2 launch ~/calibration/cuvslam_stereo.launch.py imu:=true tf:=true  # + map -> odom -> base_footprint TF
  ros2 launch docs/cuvslam_nvblox/tools/cuvslam_stereo.launch.py imu:=true tf:=true use_sim_time:=true  # laptop, bag replay
      (robot base launch with wheel_odom_tf:=false; needed by nvblox / Nav2)

imu:=true (tracking_mode 1): needs the robot base launch (robot_state_publisher gives the
camera -> imu_link TF from the URDF, d435_joint holds the Kalibr result), apply_calib_node
(/opencr/imu_corrected) and imu_time_shift.py (/opencr/imu_synced, Kalibr's 17.1 ms at IR 1280x720 applied).
The pose is reported for base_footprint, so x is the robot's forward direction.

viz:=false (default): TF publishing off. The robot's URDF already gives camera_link a parent and
diff_drive_controller owns odom -> base_footprint, so cuVSLAM publishing its own chain would give a
frame two parents. Read the result from /visual_slam/tracking/odometry (cuvslam_monitor.py).

viz:=true: cuVSLAM publishes vslam_map -> vslam_odom -> camera_link and the landmark / observation
clouds, so RViz can show the camera moving through the point map. Only while robot_state_publisher
is not running.
"""
import launch
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def is_true(context, name):
    return LaunchConfiguration(name).perform(context).lower() in ('true', '1')


def visual_slam(context):
    viz = is_true(context, 'viz')
    imu = is_true(context, 'imu')
    tf = is_true(context, 'tf')
    if tf and not imu:
        raise RuntimeError('tf:=true is for imu:=true (base_footprint); stereo-only uses viz:=true')
    publish_tf = viz or tf
    if viz and imu:
        raise RuntimeError('viz:=true needs the robot base launch off, imu:=true needs it on')

    params = {
        # Bag replay on the laptop (record_nvblox_bag.sh): follow /clock so poses carry the bag's stamps.
        'use_sim_time': is_true(context, 'use_sim_time'),
        'tracking_mode': 1 if imu else 0,
        'rectified_images': True,           # image_rect_raw from the D435
        'enable_image_denoising': False,
        'image_jitter_threshold_ms': 35.0,  # 30 fps camera: 33 ms between frames
        'base_frame': 'base_footprint' if imu else 'camera_link',
        'camera_optical_frames': [
            'camera_infra1_optical_frame',
            'camera_infra2_optical_frame',
        ],
        # With tf:=true cuVSLAM owns the standard map / odom frames of the robot.
        'map_frame': 'map' if tf else 'vslam_map',
        'odom_frame': 'odom' if tf else 'vslam_odom',
        'publish_map_to_odom_tf': publish_tf,
        'publish_odom_to_base_tf': publish_tf,
        'enable_slam_visualization': True,
        # Point clouds for RViz whenever TF is published (viz or tf), so they can be placed in the map.
        'enable_landmarks_view': publish_tf,
        'enable_observations_view': publish_tf,
    }
    remappings = [
        ('visual_slam/image_0', '/camera/camera/infra1/image_rect_raw'),
        ('visual_slam/camera_info_0', '/camera/camera/infra1/camera_info'),
        ('visual_slam/image_1', '/camera/camera/infra2/image_rect_raw'),
        ('visual_slam/camera_info_1', '/camera/camera/infra2/camera_info'),
    ]
    if imu:
        params.update({
            'imu_frame': 'imu_link',
            # Provisional noise (imu_provisional_200.yaml): datasheet x10, no Allan run yet.
            'gyro_noise_density': 0.0026,
            'gyro_random_walk': 0.00003,
            'accel_noise_density': 0.03,
            'accel_random_walk': 0.001,
            'calibration_frequency': 175.0,     # measured /imu rate (200 Hz loop with overruns)
            'imu_jitter_threshold_ms': 20.0,    # ros2_control overruns leave gaps up to ~15 ms
        })
        remappings.append(('visual_slam/imu', '/opencr/imu_synced'))

    node = ComposableNode(
        name='visual_slam_node',
        package='isaac_ros_cuvslam',
        plugin='nvidia::isaac_ros::visual_slam::VisualSlamNode',
        parameters=[params],
        remappings=remappings,
    )
    return [ComposableNodeContainer(
        name='visual_slam_launch_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[node],
        output='screen',
    )]


def generate_launch_description():
    return launch.LaunchDescription([
        DeclareLaunchArgument('viz', default_value='false',
                              description='publish TF and point clouds for RViz (robot base launch must be off)'),
        DeclareLaunchArgument('imu', default_value='false',
                              description='fuse the OpenCR IMU (tracking_mode 1); robot base launch must run'),
        DeclareLaunchArgument('tf', default_value='false',
                              description='with imu:=true, publish map -> odom -> base_footprint '
                                          '(robot base launch with wheel_odom_tf:=false)'),
        DeclareLaunchArgument('use_sim_time', default_value='false',
                              description='true when replaying a bag with ros2 bag play --clock'),
        OpaqueFunction(function=visual_slam),
    ])
