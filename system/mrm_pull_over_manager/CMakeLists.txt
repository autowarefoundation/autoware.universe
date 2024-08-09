cmake_minimum_required(VERSION 3.14)
project(mrm_pull_over_manager)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_executable(mrm_pull_over_manager
  src/mrm_pull_over_manager/mrm_pull_over_manager_node.cpp
  src/mrm_pull_over_manager/mrm_pull_over_manager_core.cpp
)

ament_auto_package(INSTALL_TO_SHARE
  launch
  config
)
