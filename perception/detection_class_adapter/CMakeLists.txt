
cmake_minimum_required(VERSION 3.5)
project(detection_class_adapter)

find_package(autoware_cmake REQUIRED)
find_package(Eigen3 REQUIRED)

autoware_package()

ament_export_include_directories(
  include
)

## node ##
ament_auto_add_library(detection_class_adapter_component SHARED
  src/detection_class_adapter.cpp
)

rclcpp_components_register_node(detection_class_adapter_component
  PLUGIN "DetectionClassAdapter"
  EXECUTABLE detection_class_adapter_node
)

ament_auto_package(
  INSTALL_TO_SHARE
  launch
  config
)
