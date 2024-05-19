cmake_minimum_required(VERSION 3.14)
project(auto_parking)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(auto_parking_node SHARED
  src/auto_parking/auto_parking_node.cpp
)

rclcpp_components_register_node(auto_parking_node
  PLUGIN "auto_parking::AutoParkingNode"
  EXECUTABLE auto_parking)

ament_auto_package(
  INSTALL_TO_SHARE
  launch
  config
)
