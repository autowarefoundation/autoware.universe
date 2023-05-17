cmake_minimum_required(VERSION 3.14)
project(behavior_velocity_planner_common)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/utilization/path_utilization.cpp
  src/utilization/util.cpp
  src/utilization/debug.cpp
)

ament_auto_package()
