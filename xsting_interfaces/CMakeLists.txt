cmake_minimum_required(VERSION 3.16)
project(xsting_interfaces)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ArmInput.msg" "msg/ArmTarget.msg" "msg/TaskState.msg"
  "action/RunTask.action" "action/MoveArm.action"
  "srv/GetTarget.srv" "srv/ReserveSlot.srv" "srv/CommitTransfer.srv" "srv/LoadModel.srv"
  DEPENDENCIES std_msgs geometry_msgs sensor_msgs builtin_interfaces)
ament_export_dependencies(rosidl_default_runtime)
if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_wire_contracts test/test_wire_contracts.py)
endif()
ament_package()
