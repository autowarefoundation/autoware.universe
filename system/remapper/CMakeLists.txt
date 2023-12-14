cmake_minimum_required(VERSION 3.14)
project(remapper)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_executable(remapper
  src/remapper_node.cpp
  src/remapper_core.cpp
)

ament_auto_package(
  INSTALL_TO_SHARE
  launch
)
