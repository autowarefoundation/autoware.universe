cmake_minimum_required(VERSION 3.14)
project(autoware_planning_factor_interface)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(autoware_planning_factor_interface SHARED
  DIRECTORY src
)

ament_auto_package()
