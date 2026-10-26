cmake_minimum_required(VERSION 3.20)
project(xsting_base_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(xsting_manipulation_hardware REQUIRED)

# 라이브러리 만들기
#   - 플러그인은 실행 파일이 아니라 controller_manager가 실행 중에 불러오는 "공유 라이브러리"
#   - 경로는 cmakelists 기준
add_library(${PROJECT_NAME} SHARED
  src/opencr_base.cpp
  src/xsting_base_system.cpp
)

# C++ 표준 지정
#   - Lyrical 최소 요구사항이 몇인지
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_20)


# 헤더 경로 지정
#   - 빌드할 때와 설치된 뒤의 include 경로가 다름
#   - 키워드: generator expression, BUILD_INTERFACE / INSTALL_INTERFACE
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

# 의존 라이브러리 연결
#   - 위 find_package로 찾은 패키지 중 ament_cmake를 뺀 5개
#   - 이름 형식: 패키지::타깃 (예: rclcpp::rclcpp)
target_link_libraries(${PROJECT_NAME} PUBLIC
  xsting_manipulation_hardware::xsting_manipulation_hardware
  hardware_interface::hardware_interface
  pluginlib::pluginlib
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
)


# 플러그인 등록
#   - "이 xml은 hardware_interface의 플러그인 목록"이라고 알리는 함수 한 줄
pluginlib_export_plugin_description_file(hardware_interface xsting_base_hardware.xml)

# 설치 규칙
#   - 무엇을 설치해야 하나? (헤더 폴더, 라이브러리)
#   - 라이브러리 설치에는 EXPORT 이름을 붙여야 ament export에서 쓸 수 있음
install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# 다른 패키지가 이 라이브러리를 쓸 수 있게 내보내기
#   - install의 EXPORT 이름과 의존 패키지 목록을 ament에 알려주기
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

# # 의존 패키지 내보내기
#   - 이 패키지를 find_package 하는 쪽에서도 아래 패키지들을 자동으로 찾게 함
#   - 우리 공개 헤더가 include 하는 패키지는 꼭 넣어야 함 (안 넣으면 쓰는 쪽에서 헤더를 못 찾음)
#   - 바로 위 ament_export_targets와 짝: 타깃은 저쪽이, 그 타깃의 의존성은 이쪽이 알려줌
ament_export_dependencies(
  xsting_manipulation_hardware # DynamixelSDKWrapper를 가져옴
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

ament_package()
