cmake_minimum_required(VERSION 3.14)
project(mission_remaining_distance_time_calculator)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(mission_remaining_distance_time_calculator SHARED
  src/mission_remaining_distance_time_calculator_node.cpp
)

rclcpp_components_register_node(mission_remaining_distance_time_calculator
  PLUGIN "mission_remaining_distance_time_calculator::MissionRemainingDistanceTimeCalculatorNode"
  EXECUTABLE mission_remaining_distance_time_calculator_node
)


ament_auto_package(
  INSTALL_TO_SHARE
  launch
  config
)
