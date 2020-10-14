cmake_minimum_required(VERSION 3.5)
project(shift_decider)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

ament_auto_add_executable(shift_decider src/shift_decider.cpp src/main.cpp)

ament_auto_package(
  INSTALL_TO_SHARE
  launch
  )
