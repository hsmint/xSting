cmake_minimum_required(VERSION 3.20)
project(xsting_bringup)

find_package(ament_cmake REQUIRED)

# 런치 파일과 설정 파일 설치
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
