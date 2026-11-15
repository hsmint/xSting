cmake_minimum_required(VERSION 3.20)
project(xsting_nav)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(xsting_interfaces REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(nvblox_msgs QUIET)

add_library(xsting_nav_core SHARED
  src/nav_core.cpp
)
target_include_directories(xsting_nav_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(xsting_nav_core PUBLIC yaml-cpp)

add_executable(nav_node
  src/nav_node.cpp
)
target_link_libraries(nav_node PRIVATE
  xsting_nav_core
  ament_index_cpp::ament_index_cpp
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  tf2::tf2
  tf2_ros::tf2_ros
  ${geometry_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${std_msgs_TARGETS}
  ${std_srvs_TARGETS}
  ${vision_msgs_TARGETS}
  ${xsting_interfaces_TARGETS}
)

add_executable(calib_node
  src/calib_node.cpp
)
target_link_libraries(calib_node PRIVATE
  xsting_nav_core
  ament_index_cpp::ament_index_cpp
  rclcpp::rclcpp
  ${nav_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${std_msgs_TARGETS}
)

if(nvblox_msgs_FOUND)
  message(STATUS "[xsting_nav] Found nvblox_msgs: enabling native DistanceMapSlice subscription")
  target_compile_definitions(nav_node PRIVATE XSTING_NAV_HAS_NVBLOX_MSGS=1)
  target_compile_definitions(calib_node PRIVATE XSTING_NAV_HAS_NVBLOX_MSGS=1)
  target_link_libraries(nav_node PRIVATE ${nvblox_msgs_TARGETS})
  target_link_libraries(calib_node PRIVATE ${nvblox_msgs_TARGETS})
else()
  message(STATUS "[xsting_nav] nvblox_msgs not found: using LaserScan/OccupancyGrid ESDF fallback")
endif()

if(BUILD_TESTING)
  add_executable(test_nav_core
    test/test_nav_core.cpp
  )
  target_link_libraries(test_nav_core xsting_nav_core)
  target_compile_definitions(test_nav_core PRIVATE
    XSTING_TEST_PKG_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
  )
  add_test(NAME test_nav_core COMMAND test_nav_core)
endif()

install(TARGETS
  xsting_nav_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  nav_node
  calib_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  include/
  DESTINATION include
)

install(DIRECTORY
  config
  launch
  models
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/calib_nav_wizard.py
  scripts/export_policy_and_map.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(xsting_nav_core)
ament_package()
