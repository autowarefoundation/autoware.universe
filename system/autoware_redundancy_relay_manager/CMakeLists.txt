cmake_minimum_required(VERSION 3.14)
project(autoware_redundancy_relay_manager)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/redundancy_relay_manager_node.cpp
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::redundancy_relay_manager::RedundancyRelayManager"
  EXECUTABLE ${PROJECT_NAME}_node
  EXECUTOR MultiThreadedExecutor
)

ament_auto_package(INSTALL_TO_SHARE
  launch
  config
)
