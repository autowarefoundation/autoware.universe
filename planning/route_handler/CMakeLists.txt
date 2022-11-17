cmake_minimum_required(VERSION 3.14)
project(route_handler)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(route_handler SHARED
  src/lanelet_point.cpp
  src/lanelet_path.cpp
  src/lanelet_route.cpp
  src/lanelet_route_builder.cpp
  src/lanelet_section.cpp
  src/route_handler.cpp
  src/utils.cpp
)

ament_auto_package()
