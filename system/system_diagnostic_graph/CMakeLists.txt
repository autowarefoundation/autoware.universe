cmake_minimum_required(VERSION 3.14)
project(system_diagnostic_graph)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_executable(${PROJECT_NAME}
  src/main.cpp
  src/lib/config.cpp
  src/lib/debug.cpp
  src/lib/graph.cpp
  src/lib/node.cpp
  src/lib/expr.cpp
)

install(
  PROGRAMS test/diagnostics.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_auto_package(INSTALL_TO_SHARE config launch)
