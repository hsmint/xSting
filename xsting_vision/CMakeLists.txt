cmake_minimum_required(VERSION 3.20)
project(xsting_vision)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# 패키지 의존성 탐색
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(composition_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(isaac_ros_visual_slam_interfaces REQUIRED)
find_package(message_filters REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc calib3d)
find_package(ZXing REQUIRED)
find_package(realsense2 REQUIRED)
find_package(realsense2_camera_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(CUDAToolkit REQUIRED)
find_package(cuda_buffer REQUIRED)
find_package(xsting_interfaces REQUIRED)

# RealSense 카메라 노드 컴포넌트 (+ 단독 실행 파일 realsense_camera)
add_library(realsense_camera_component SHARED
  src/realsense_camera_node.cc
)
target_include_directories(realsense_camera_component PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(realsense_camera_component PRIVATE
  ament_index_cpp::ament_index_cpp
  rclcpp::rclcpp
  rclcpp_components::component
  ${sensor_msgs_TARGETS}
  ${realsense2_camera_msgs_TARGETS}
  realsense2::realsense2
  CUDA::cudart
  CUDA::nvjpeg
  cuda_buffer::cuda_buffer
)
rclcpp_components_register_node(realsense_camera_component
  PLUGIN "xsting_vision::RealsenseCameraNode"
  EXECUTABLE realsense_camera
)

# RealSense 에미터 on/off 분리 노드 컴포넌트 (+ 실행 파일 realsense_splitter)
add_library(realsense_splitter_component SHARED
  src/realsense_splitter_node.cc
)
target_include_directories(realsense_splitter_component PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(realsense_splitter_component PRIVATE
  ament_index_cpp::ament_index_cpp
  rclcpp::rclcpp
  rclcpp_components::component
  ${sensor_msgs_TARGETS}
  ${realsense2_camera_msgs_TARGETS}
  message_filters::message_filters
)
rclcpp_components_register_node(realsense_splitter_component
  PLUGIN "xsting_vision::RealsenseSplitterNode"
  EXECUTABLE realsense_splitter
)

# Arducam UC-684 카메라 노드 컴포넌트 (+ 단독 실행 파일 arducam_camera)
add_library(arducam_camera_component SHARED
  src/arducam_camera_node.cc
)
target_include_directories(arducam_camera_component PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(arducam_camera_component PRIVATE
  ament_index_cpp::ament_index_cpp
  rclcpp::rclcpp
  rclcpp_components::component
  ${sensor_msgs_TARGETS}
  CUDA::cudart
  CUDA::nvjpeg
  CUDA::nppig
  cuda_buffer::cuda_buffer
)
rclcpp_components_register_node(arducam_camera_component
  PLUGIN "xsting_vision::ArducamCameraNode"
  EXECUTABLE arducam_camera
)

# Depth 전경 점군 통합 PCA 3D 검출 노드 컴포넌트 (+ 단독 실행 파일 depth_pca)
add_library(depth_pca_component SHARED
  src/depth_pca_node.cc
)
target_include_directories(depth_pca_component PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(depth_pca_component PRIVATE
  ament_index_cpp::ament_index_cpp
  rclcpp::rclcpp
  rclcpp_components::component
  ${geometry_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${vision_msgs_TARGETS}
  tf2::tf2
  tf2_ros::tf2_ros
  opencv_core
  CUDA::cudart
  cuda_buffer::cuda_buffer
)
rclcpp_components_register_node(depth_pca_component
  PLUGIN "xsting_vision::DepthPcaNode"
  EXECUTABLE depth_pca
)

# 체커보드 기반 두 카메라 외부 파라미터 1회성 보정 노드 (+ 실행 파일 cam_calib)
add_library(cam_calib_component SHARED
  src/cam_calib_node.cc
)
target_include_directories(cam_calib_component PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(cam_calib_component PRIVATE
  ament_index_cpp::ament_index_cpp
  rclcpp::rclcpp
  rclcpp_components::component
  ${geometry_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  tf2::tf2
  tf2_ros::tf2_ros
  opencv_core
  opencv_imgproc
  opencv_calib3d
  CUDA::cudart
  cuda_buffer::cuda_buffer
)
rclcpp_components_register_node(cam_calib_component
  PLUGIN "xsting_vision::CamCalibNode"
  EXECUTABLE cam_calib
)

# 신호등 점멸 순서 및 Code 128 바코드 Action 인식 노드 (+ 실행 파일 sign_barcode)
add_library(sign_barcode_component SHARED
  src/sign_barcode_node.cc
)
target_include_directories(sign_barcode_component PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(sign_barcode_component PRIVATE
  ament_index_cpp::ament_index_cpp
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  rclcpp_components::component
  ${composition_interfaces_TARGETS}
  ${sensor_msgs_TARGETS}
  ${vision_msgs_TARGETS}
  ${xsting_interfaces_TARGETS}
  ZXing::ZXing
  CUDA::cudart
  CUDA::nppicc
  cuda_buffer::cuda_buffer
)
rclcpp_components_register_node(sign_barcode_component
  PLUGIN "xsting_vision::SignBarcodeNode"
  EXECUTABLE sign_barcode
)

# 1층 중앙 2D LiDAR 본체·다리 마스킹 + ICP 오도메트리 보정 + nvblox 점군 변환 컴포넌트
add_library(lidar_odom_component SHARED
  src/lidar_odom_node.cc
)
target_include_directories(lidar_odom_component PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(lidar_odom_component PRIVATE
  ament_index_cpp::ament_index_cpp
  rclcpp::rclcpp
  rclcpp_components::component
  ${geometry_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${isaac_ros_visual_slam_interfaces_TARGETS}
  tf2::tf2
  tf2_ros::tf2_ros
)
rclcpp_components_register_node(lidar_odom_component
  PLUGIN "xsting_vision::LidarOdomNode"
  EXECUTABLE lidar_odom
)

# 스캔 직선 피팅 및 RealSense 깊이 정합 기반 LiDAR 외부 파라미터 보정 노드
add_library(lidar_calib_component SHARED
  src/lidar_calib_node.cc
)
target_include_directories(lidar_calib_component PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(lidar_calib_component PRIVATE
  ament_index_cpp::ament_index_cpp
  rclcpp::rclcpp
  rclcpp_components::component
  ${geometry_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  tf2::tf2
  tf2_ros::tf2_ros
  CUDA::cudart
  cuda_buffer::cuda_buffer
)
rclcpp_components_register_node(lidar_calib_component
  PLUGIN "xsting_vision::LidarCalibNode"
  EXECUTABLE lidar_calib
)

# 컴포넌트 라이브러리, YAML 설정 파일, TensorRT 엔진 설치
install(TARGETS
  realsense_camera_component
  realsense_splitter_component
  arducam_camera_component
  depth_pca_component
  cam_calib_component
  sign_barcode_component
  lidar_odom_component
  lidar_calib_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# 빌드 플랫폼(arm64 / x86_64)에 맞는 TensorRT 엔진과 공용 ONNX 설치
if(CMAKE_SYSTEM_PROCESSOR MATCHES "^(aarch64|arm64)$")
  set(MODEL_ARCH "arm64")
else()
  set(MODEL_ARCH "x86_64")
endif()
install(DIRECTORY model/${MODEL_ARCH} model/onnx
  DESTINATION share/${PROJECT_NAME}/model
)

ament_package()
