cmake_minimum_required(VERSION 3.14)
project(object_filter)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(Boost REQUIRED)

include_directories(
  include
  SYSTEM
  ${Boost_INCLUDE_DIRS}
)

ament_auto_add_library(object_filter SHARED
  src/detected_object_filter.cpp
)

rclcpp_components_register_node(object_filter
  PLUGIN "detected_object_filter::DetectedObjectFilterNode"
  EXECUTABLE detected_object_filter
)

ament_auto_package(INSTALL_TO_SHARE
  launch
)
