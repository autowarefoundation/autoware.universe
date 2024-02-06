cmake_minimum_required(VERSION 3.14)
project(map_height_fitter)

find_package(autoware_cmake REQUIRED)
find_package(PCL REQUIRED COMPONENTS common)
autoware_package()

ament_auto_add_library(map_height_fitter SHARED
  src/map_height_fitter.cpp
)
target_link_libraries(map_height_fitter ${PCL_LIBRARIES})
target_compile_options(map_height_fitter PRIVATE -Wno-pedantic)

ament_auto_add_executable(node
  src/node.cpp
)

ament_auto_package(INSTALL_TO_SHARE launch)
