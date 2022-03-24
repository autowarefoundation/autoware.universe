cmake_minimum_required(VERSION 3.5)
project(ndt_scan_matcher)

if(${CMAKE_SYSTEM_PROCESSOR} MATCHES "arm")
  if(BUILD_WITH_MARCH_NATIVE)
    add_compile_options(-march=native)
  endif()
else()
  add_compile_options(-msse -msse2 -msse3 -msse4 -msse4.1 -msse4.2)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_executable(ndt_scan_matcher
  src/debug.cpp
  src/ndt_scan_matcher_node.cpp
  src/ndt_scan_matcher_core.cpp
  src/util_func.cpp
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
  INSTALL_TO_SHARE
    launch
    config
)
