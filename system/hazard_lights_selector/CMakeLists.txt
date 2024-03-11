cmake_minimum_required(VERSION 3.14)
project(hazard_lights_selector)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(hazard_lights_selector_node SHARED
  src/hazard_lights_selector_node.cpp
)

rclcpp_components_register_node(hazard_lights_selector_node
  PLUGIN "hazard_lights_selector::HazardLightsSelectorNode"
  EXECUTABLE hazard_lights_selector
)

ament_auto_package(INSTALL_TO_SHARE
  launch
  config
)
