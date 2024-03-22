cmake_minimum_required(VERSION 3.14)
project(mrm_emergency_stop_operator)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(mrm_emergency_stop_operator SHARED
  src/mrm_emergency_stop_operator.cpp
)

ament_auto_package(INSTALL_TO_SHARE
  launch
  config
)
