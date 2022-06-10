cmake_minimum_required(VERSION 3.14)
project(acceleration_estimator)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_executable(acceleration_estimator
  src/acceleration_estimator_node.cpp
  src/acceleration_estimator.cpp
)
ament_target_dependencies(acceleration_estimator)

ament_auto_package(
  INSTALL_TO_SHARE
  launch
)
