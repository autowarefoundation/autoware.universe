cmake_minimum_required(VERSION 3.5)
project(autoware_debug_msgs)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BoolStamped.msg"
  "msg/Float32Stamped.msg"
  "msg/Float32MultiArrayStamped.msg"
  "msg/Float64Stamped.msg"
  "msg/Float64MultiArrayStamped.msg"
  "msg/Int32Stamped.msg"
  "msg/Int32MultiArrayStamped.msg"
  "msg/Int64Stamped.msg"
  "msg/Int64MultiArrayStamped.msg"
  "msg/StringStamped.msg"
  DEPENDENCIES
    builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
