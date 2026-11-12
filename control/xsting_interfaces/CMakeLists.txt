cmake_minimum_required(VERSION 3.8)
project(xsting_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ErrorCode.msg"
  "msg/PuckObservation.msg"
  "msg/CellObservation.msg"
  "msg/StorageSlot.msg"
  "action/RobotCommand.action"
  "action/RunTask.action"
  "action/WatchSignal.action"
  "srv/ReadBarcode.srv"
  "srv/DetectPucks.srv"
  "srv/ObserveCells.srv"
  "srv/GetStorage.srv"
  DEPENDENCIES geometry_msgs builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
