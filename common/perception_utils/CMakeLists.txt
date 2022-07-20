cmake_minimum_required(VERSION 3.14)
project(perception_utils)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(Boost REQUIRED)

ament_auto_add_library(perception_utils SHARED
  src/perception_utils.cpp
)

ament_auto_package()
