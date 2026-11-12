cmake_minimum_required(VERSION 3.8)
project(so100_interfaces)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ArmState.msg"
  "action/Pick.action"
  "action/Place.action"
  "action/PickPlace.action"
  "action/Home.action"
  "action/Rest.action"
  DEPENDENCIES geometry_msgs std_msgs)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
