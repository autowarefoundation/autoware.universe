cmake_minimum_required(VERSION 3.14)

project(recordreplay_planner)

### Dependencies
find_package(autoware_cmake REQUIRED)
find_package(tf2 REQUIRED)
find_package(geometry_msgs REQUIRED)

autoware_package()

# Build library
ament_auto_add_library(${PROJECT_NAME} SHARED
  src/recordreplay_planner/recordreplay_planner.cpp
)

ament_auto_package(INSTALL_TO_SHARE)
