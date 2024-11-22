cmake_minimum_required(VERSION 3.14)

project(autoware_pyplot)

option(BUILD_EXAMPLES "Build examples" OFF)

find_package(autoware_cmake REQUIRED)

find_package(
  Python3
  COMPONENTS Interpreter Development
  REQUIRED)

autoware_package()

ament_auto_add_library(${PROJECT_NAME} SHARED
  DIRECTORY src
)
target_link_libraries(${PROJECT_NAME} ${Python3_LIBRARIES} pybind11::embed)

ament_auto_package()
