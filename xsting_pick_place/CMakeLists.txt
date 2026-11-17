cmake_minimum_required(VERSION 3.20)
project(xsting_pick_place)

# C++17 및 컴파일 경고 설정
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 필수 ROS 2, MoveIt 및 TensorRT 의존성 탐색
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(control_msgs REQUIRED)
find_package(cuda_buffer REQUIRED)
find_package(CUDAToolkit REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(xsting_interfaces REQUIRED)

find_library(NVINFER_LIB nvinfer REQUIRED)
find_library(NVONNXPARSER_LIB nvonnxparser REQUIRED)

# 기구학·비전·미션 규칙 공통 정적 라이브러리 빌드
add_library(pick_core STATIC
  src/arducam_align.cc
  src/arm_kin.cc
  src/mission_rules.cc
  src/motion_ctrl.cc
  src/puck_det.cc
)
target_include_directories(pick_core PUBLIC include)
target_link_libraries(pick_core PUBLIC
  ament_index_cpp::ament_index_cpp
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  CUDA::cudart
  cuda_buffer::cuda_buffer
  ${NVINFER_LIB}
  ${NVONNXPARSER_LIB}
  opencv_core
  opencv_imgproc
  ${control_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${moveit_msgs_TARGETS}
  ${moveit_ros_planning_interface_TARGETS}
  ${sensor_msgs_TARGETS}
  ${shape_msgs_TARGETS}
  ${trajectory_msgs_TARGETS}
  ${vision_msgs_TARGETS}
)

# cuMotion + MoveIt 기반 픽앤플레이스 액션 노드 빌드
add_executable(pick_place_node src/pick_place_node.cc)
target_link_libraries(pick_place_node PRIVATE
  pick_core
  ${xsting_interfaces_TARGETS}
)

# 실기체 ForwardCommandController 연동용 궤적 브리지 노드 빌드
add_executable(traj_bridge_node src/traj_bridge_node.cc)
target_include_directories(traj_bridge_node PRIVATE include)
target_link_libraries(traj_bridge_node PRIVATE
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  ${control_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${std_msgs_TARGETS}
  ${trajectory_msgs_TARGETS}
)

# 실행 파일·설정·런치·스크립트 설치
install(TARGETS pick_place_node traj_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include
)
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)
install(PROGRAMS
  scripts/send_mission.py
  scripts/calib_pick_place.py
  scripts/setup_isaac_scene.py
  scripts/run_isaac_sim.sh
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  enable_testing()
  add_executable(test_mission_rules
    test/test_mission_rules.cc
    src/mission_rules.cc
    src/arm_kin.cc
  )
  target_include_directories(test_mission_rules PRIVATE include)
  add_test(NAME test_mission_rules COMMAND test_mission_rules)
endif()

ament_package()
