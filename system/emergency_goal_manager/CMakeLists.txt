cmake_minimum_required(VERSION 3.14)
project(emergency_goal_manager)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_executable(emergency_goal_manager
  src/emergency_goal_manager_node.cpp
  src/emergency_goal_manager_core.cpp
)

ament_auto_package(INSTALL_TO_SHARE
  launch
  config
)
