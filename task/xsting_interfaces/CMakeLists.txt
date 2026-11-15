cmake_minimum_required(VERSION 3.20)
project(xsting_interfaces)

# 패키지 의존성 탐색
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)

# Msg 및 Action 인터페이스 코드 생성
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CellObservation.msg"
  "msg/ErrorCode.msg"
  "msg/LoadedPuck.msg"
  "msg/PlaceItem.msg"
  "msg/PlacedItem.msg"
  "action/ApproachBox.action"
  "action/LoadBox.action"
  "action/ObserveLocation.action"
  "action/PickPlace.action"
  "action/ReadMissionInput.action"
  "action/ReadSignBarcode.action"
  "action/RobotCommand.action"
  "action/RunTask.action"
  "action/UnloadPucks.action"
  DEPENDENCIES builtin_interfaces geometry_msgs
)

# 런타임 의존성 내보내기
ament_export_dependencies(rosidl_default_runtime)
ament_package()
