cmake_minimum_required(VERSION 3.5)
project(spline_interpolation)

if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()


ament_auto_add_library(spline_interpolation src/spline_interpolation.cpp)

ament_auto_package()