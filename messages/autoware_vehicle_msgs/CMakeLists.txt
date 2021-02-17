cmake_minimum_required(VERSION 3.5)
project(autoware_vehicle_msgs)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(autoware_control_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Engage.msg"
  "msg/VehicleCommand.msg"
  "msg/Shift.msg"
  "msg/ShiftStamped.msg"
  "msg/ControlMode.msg"
  "msg/TurnSignal.msg"
  "msg/Steering.msg"
  "msg/RawVehicleCommand.msg"
  "msg/RawControlCommand.msg"
  "msg/RawControlCommandStamped.msg"
  DEPENDENCIES std_msgs geometry_msgs autoware_control_msgs
)

ament_package()
