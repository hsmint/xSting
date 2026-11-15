cmake_minimum_required(VERSION 3.16)
project(xsting_moveit_config)
find_package(ament_cmake REQUIRED)
install(DIRECTORY config launch scripts DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE)
ament_package()
