cmake_minimum_required(VERSION 3.14)
project(system_diagnostic_graph)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_executable(${PROJECT_NAME}
  src/node.cpp
)

ament_auto_package(INSTALL_TO_SHARE config launch)
