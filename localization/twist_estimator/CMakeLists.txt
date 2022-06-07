cmake_minimum_required(VERSION 3.14)
project(twist_estimator)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_executable(${PROJECT_NAME}
  src/twist_estimator_node.cpp
  src/twist_estimator_core.cpp
)

ament_auto_package(INSTALL_TO_SHARE
  launch
)
