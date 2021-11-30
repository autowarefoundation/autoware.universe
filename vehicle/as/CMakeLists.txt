cmake_minimum_required(VERSION 3.5)
project(as)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wno-unused-parameter -Wall -Wextra -Wpedantic)
endif()

ament_auto_add_executable(ssc_interface
  src/ssc_interface/ssc_interface_node.cpp
  src/ssc_interface/ssc_interface.cpp
)

ament_auto_add_executable(pacmod_interface
  src/pacmod_interface/pacmod_interface.cpp
  src/pacmod_interface/pacmod_interface_node.cpp
)

ament_auto_add_executable(pacmod_additional_debug_publisher
  src/pacmod_additional_debug_publisher/pacmod_additional_debug_publisher_node.cpp
  src/pacmod_additional_debug_publisher/pacmod_additional_debug_publisher_main.cpp
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
