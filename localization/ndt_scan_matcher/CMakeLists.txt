cmake_minimum_required(VERSION 3.5)
project(ndt_scan_matcher)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_executable(ndt_scan_matcher
  src/ndt_scan_matcher_node.cpp
  src/ndt_scan_matcher_core.cpp
)

ament_auto_package(INSTALL_TO_SHARE launch)
