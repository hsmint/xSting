cmake_minimum_required(VERSION 3.16)
project(xsting_description)

find_package(ament_cmake REQUIRED)

# 빌드할 코드는 없고, 아래 폴더를 share/xsting_description 으로 복사만 한다.
# URDF 안의 package://xsting_description/meshes/... 경로가 이 설치 위치를 가리킨다.
install(
  DIRECTORY config launch meshes rviz urdf usd scripts
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
