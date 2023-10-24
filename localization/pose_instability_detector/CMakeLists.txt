cmake_minimum_required(VERSION 3.14)
project(pose_instability_detector)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_executable(pose_instability_detector
  src/pose_instability_detector_node.cpp
  src/pose_instability_detector_core.cpp
)

ament_auto_package(
  INSTALL_TO_SHARE
    launch
    config
)
