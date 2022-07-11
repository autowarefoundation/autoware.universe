cmake_minimum_required(VERSION 3.14)
project(mrm_comfortable_stop_operator)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_executable(mrm_comfortable_stop_operator
  src/mrm_comfortable_stop_operator/mrm_comfortable_stop_operator_node.cpp
  src/mrm_comfortable_stop_operator/mrm_comfortable_stop_operator_core.cpp
)

ament_auto_package(INSTALL_TO_SHARE
  launch
  config
)
