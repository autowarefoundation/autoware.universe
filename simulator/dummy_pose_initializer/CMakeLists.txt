cmake_minimum_required(VERSION 3.14)
project(dummy_pose_initializer)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_executable(${PROJECT_NAME}
  src/dummy_pose_initializer_core.cpp
  src/dummy_pose_initializer_node.cpp
)

ament_auto_package(INSTALL_TO_SHARE launch)
