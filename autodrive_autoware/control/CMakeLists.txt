cmake_minimum_required(VERSION 3.14)
project(autodrive_trajectory_follower)

find_package(autoware_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)
autoware_package()

set(CONTROLLER_NODE controller_node)
ament_auto_add_library(${CONTROLLER_NODE} SHARED
  include/controller_node.hpp
  src/controller_node.cpp
)

target_compile_options(${CONTROLLER_NODE} PRIVATE -Wno-error=old-style-cast)
rclcpp_components_register_node(${CONTROLLER_NODE}
  PLUGIN "autoware::motion::control::trajectory_follower_nodes::Controller"
  EXECUTABLE ${CONTROLLER_NODE}_exe
)

set(AUTODRIVE_TRAJECTORY_FOLLOWER_NODE autodrive_trajectory_follower)
ament_auto_add_library(${AUTODRIVE_TRAJECTORY_FOLLOWER_NODE} SHARED
  include/autodrive_trajectory_follower.hpp
  src/autodrive_trajectory_follower.cpp
)

ament_target_dependencies(${AUTODRIVE_TRAJECTORY_FOLLOWER_NODE} ackermann_msgs)

target_compile_options(${AUTODRIVE_TRAJECTORY_FOLLOWER_NODE} PRIVATE -Wno-error=old-style-cast)
rclcpp_components_register_node(${AUTODRIVE_TRAJECTORY_FOLLOWER_NODE}
  PLUGIN "autodrive_trajectory_follower::AutoDRIVETrajectoryFollower"
  EXECUTABLE ${AUTODRIVE_TRAJECTORY_FOLLOWER_NODE}_exe
)

ament_auto_package(
  INSTALL_TO_SHARE
  config
  launch
)