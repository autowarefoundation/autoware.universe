cmake_minimum_required(VERSION 3.14)
project(emergency_goal_manager)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(emergency_goal_manager_component SHARED
  src/emergency_goal_manager_node.cpp
)

rclcpp_components_register_node(emergency_goal_manager_component
  PLUGIN "emergency_goal_manager::EmergencyGoalManager"
  EXECUTABLE emergency_goal_manager)

ament_auto_package(INSTALL_TO_SHARE
  launch
  config
)
