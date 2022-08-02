cmake_minimum_required(VERSION 3.14)
project(default_ad_api_helpers)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_executable(automatic_pose_initializer
  src/automatic_pose_initializer.cpp
)

ament_auto_add_executable(initial_pose_adaptor
  src/initial_pose_adaptor.cpp
)

ament_auto_package(INSTALL_TO_SHARE config launch)
