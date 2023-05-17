cmake_minimum_required(VERSION 3.14)
project(behavior_velocity_stop_line_module)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/manager.cpp
  src/scene.cpp
  src/debug.cpp
)

ament_auto_package()
