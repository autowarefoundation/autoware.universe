cmake_minimum_required(VERSION 3.14)
project(localization_util)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(util_func SHARED
  src/util_func.cpp
)

ament_auto_add_library(pose_array_interpolator SHARED
  src/pose_array_interpolator.cpp
)

ament_auto_add_library(tf2_listener_module SHARED
  src/tf2_listener_module.cpp
)

ament_auto_package(
  INSTALL_TO_SHARE
)
