cmake_minimum_required(VERSION 3.5)
project(pose_initializer)

### Compile options
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
  add_compile_options(-Wno-unused-parameter)
endif()

### Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(autoware_localization_srvs REQUIRED)

### Includes
include_directories(include)

add_executable(pose_initializer
  src/pose_initializer_node.cpp
  src/pose_initializer_core.cpp
)

set(POSE_INITIALIZER_DEPENDENCIES
  rclcpp
  tf2
  tf2_ros
  tf2_geometry_msgs
  std_msgs
  geometry_msgs
  sensor_msgs
  PCL
  pcl_conversions
  autoware_localization_srvs
)

ament_target_dependencies(pose_initializer ${POSE_INITIALIZER_DEPENDENCIES})
ament_export_dependencies(${POSE_INITIALIZER_DEPENDENCIES})

install(
  TARGETS pose_initializer
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY
    launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
