cmake_minimum_required(VERSION 3.5)
project(autoware_system_msgs)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/AutowareState.msg"
  "msg/DrivingCapability.msg"
  "msg/HazardStatus.msg"
  "msg/HazardStatusStamped.msg"
  DEPENDENCIES diagnostic_msgs std_msgs
)

ament_package()
