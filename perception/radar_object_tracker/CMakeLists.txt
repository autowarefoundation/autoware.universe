cmake_minimum_required(VERSION 3.8)
project(radar_object_tracker)

find_package(autoware_cmake REQUIRED)
autoware_package()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

### Find Eigen Dependencies
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
  SYSTEM
    ${EIGEN3_INCLUDE_DIR}
)

# Targets
ament_auto_add_library(radar_object_tracker_node_component SHARED
  src/radar_object_tracker_node/radar_object_tracker_node.cpp
)

rclcpp_components_register_node(radar_object_tracker_node_component
  PLUGIN "radar_object_clustering::RadarObjectTrackerNode"
  EXECUTABLE radar_object_tracker_node
)

# testing
# todo

# Package
ament_auto_package(
  INSTALL_TO_SHARE
  launch
  config
)
