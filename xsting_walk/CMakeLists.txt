cmake_minimum_required(VERSION 3.20)
project(xsting_walk)

# C++20 표준 및 컴파일 경고 설정
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 필수 ROS 2 및 수치 라이브러리 탐색
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

# 순수 C++20 보행 신경망(74->18)·실측 보정·CPG·기구학 정적 라이브러리
add_library(walk_core STATIC
  src/mlp.cc
  src/calib.cc
  src/walk_ctrl.cc
)
target_include_directories(walk_core PUBLIC include)
target_link_libraries(walk_core PUBLIC
  Eigen3::Eigen
  yaml-cpp
)

set(ROS_DEPS
  walk_core
  ament_index_cpp::ament_index_cpp
  rclcpp::rclcpp
  ${diagnostic_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${std_msgs_TARGETS}
  ${std_srvs_TARGETS}
  ${visualization_msgs_TARGETS}
)

# 전방향 보행 제어 노드, C++ 키보드 텔레옵 노드, 실측 보정 노드 빌드
add_executable(walk_node src/walk_node.cc)
target_link_libraries(walk_node PRIVATE ${ROS_DEPS})

add_executable(teleop_key_node src/teleop_key_node.cc)
target_link_libraries(teleop_key_node PRIVATE ${ROS_DEPS})

add_executable(calib_imu_node src/calib_imu_node.cc)
target_link_libraries(calib_imu_node PRIVATE ${ROS_DEPS})

add_executable(calib_joints_node src/calib_joints_node.cc)
target_link_libraries(calib_joints_node PRIVATE ${ROS_DEPS})

add_executable(calib_gait_node src/calib_gait_node.cc)
target_link_libraries(calib_gait_node PRIVATE ${ROS_DEPS})

# CTest 정합성 유닛 테스트 등록
if(BUILD_TESTING)
  enable_testing()
  add_executable(test_walk_core test/test_walk_core.cc)
  target_compile_definitions(test_walk_core PRIVATE
    XSTING_WALK_MODEL_BIN="${CMAKE_CURRENT_SOURCE_DIR}/models/walk_policy_74d.bin"
  )
  target_link_libraries(test_walk_core PRIVATE walk_core)
  add_test(NAME test_walk_core COMMAND test_walk_core)
endif()

# 실행 파일·헤더·설정·런치·모델·스크립트 설치
install(TARGETS
  walk_node
  teleop_key_node
  calib_imu_node
  calib_joints_node
  calib_gait_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include
)
install(DIRECTORY config launch models
  DESTINATION share/${PROJECT_NAME}
)
install(FILES walk_calib_guide.html
  DESTINATION share/${PROJECT_NAME}
)
install(PROGRAMS
  scripts/calib_walk_wizard.sh
  scripts/export_walk_policy.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
