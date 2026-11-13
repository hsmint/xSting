cmake_minimum_required(VERSION 3.8)
project(xsting_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ErrorCode.msg"
  "msg/CellObservation.msg"
  "msg/LoadedPuck.msg"
  "msg/PlaceItem.msg"
  "msg/PlacedItem.msg"
  "action/RobotCommand.action"
  "action/RunTask.action"
  "action/ReadMissionInput.action"
  "action/LoadBox.action"
  "action/UnloadPucks.action"
  "action/ObserveLocation.action"
  DEPENDENCIES geometry_msgs builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
