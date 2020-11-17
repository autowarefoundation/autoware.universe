cmake_minimum_required(VERSION 3.5)
project(map_based_prediction)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
  add_compile_options(-Wno-unused-parameter)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(Eigen3 REQUIRED)

ament_auto_add_executable(map_based_prediction
  src/map_based_prediction_main.cpp
  src/map_based_prediction_ros.cpp
  src/map_based_prediction.cpp
)

ament_auto_package(
  INSTALL_TO_SHARE
  launch
) 