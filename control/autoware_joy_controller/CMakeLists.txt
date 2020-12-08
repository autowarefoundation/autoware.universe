cmake_minimum_required(VERSION 3.5)
project(autoware_joy_controller)

if(NOT CMAKE_CXX_STANDARD)
	set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-unused-parameter)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_executable(autoware_joy_controller
  src/autoware_joy_controller/main.cpp
  src/autoware_joy_controller/autoware_joy_controller_node.cpp
)

ament_auto_package(
  INSTALL_TO_SHARE
  launch
  config
)
