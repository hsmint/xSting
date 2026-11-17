cmake_minimum_required(VERSION 3.20)
project(xsting_approach)

# C++20 표준 및 컴파일 경고 설정
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 필수 ROS 2 및 수치 라이브러리 탐색
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(xsting_interfaces REQUIRED)
find_package(yaml-cpp REQUIRED)

# 순수 C++ 신경망·보정·PCA·제어 코어 정적 라이브러리 빌드
add_library(approach_core STATIC
  src/mlp.cc
  src/calib.cc
  src/pca_vis.cc
  src/approach_ctrl.cc
)
target_include_directories(approach_core PUBLIC include)
target_link_libraries(approach_core PUBLIC
  Eigen3::Eigen
  yaml-cpp
)

# ROS 2 메시지 및 Foxglove 시각화 정적 라이브러리 빌드
set(ROS_DEPS
  approach_core
  ament_index_cpp::ament_index_cpp
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  ${geometry_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${std_msgs_TARGETS}
  ${std_srvs_TARGETS}
  ${vision_msgs_TARGETS}
  ${visualization_msgs_TARGETS}
  ${xsting_interfaces_TARGETS}
)

add_library(approach_vis STATIC src/foxglove_vis.cc)
target_include_directories(approach_vis PUBLIC include)
target_link_libraries(approach_vis PUBLIC ${ROS_DEPS})

# 정밀 접근 액션·제어 노드 및 실기 보정 노드 빌드
add_executable(approach_node src/approach_node.cc)
target_link_libraries(approach_node PRIVATE approach_vis ${ROS_DEPS})

add_executable(calib_joints_node src/calib_joints_node.cc)
target_link_libraries(calib_joints_node PRIVATE ${ROS_DEPS})

add_executable(calib_imu_node src/calib_imu_node.cc)
target_link_libraries(calib_imu_node PRIVATE ${ROS_DEPS})

add_executable(calib_cam_node src/calib_cam_node.cc)
target_link_libraries(calib_cam_node PRIVATE approach_vis ${ROS_DEPS})

# 실행 파일·헤더·설정·모델·스크립트 설치
install(TARGETS
  approach_node
  calib_joints_node
  calib_imu_node
  calib_cam_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include
)
install(DIRECTORY config launch models
  DESTINATION share/${PROJECT_NAME}
)
install(PROGRAMS scripts/calib_wizard.sh
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
