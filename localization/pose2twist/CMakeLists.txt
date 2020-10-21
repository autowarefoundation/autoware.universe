cmake_minimum_required(VERSION 3.5)
project(pose2twist)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_executable(pose2twist
  src/pose2twist_node.cpp
  src/pose2twist_core.cpp
)

ament_auto_package(
  INSTALL_TO_SHARE
  launch
)
