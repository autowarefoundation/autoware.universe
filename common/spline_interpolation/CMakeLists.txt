cmake_minimum_required(VERSION 3.0.2)
project(spline_interpolation)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    spline_interpolation
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(spline_interpolation src/spline_interpolation.cpp)

target_link_libraries(spline_interpolation
  ${catkin_LIBRARIES}
)

add_dependencies(spline_interpolation
  ${catkin_EXPORTED_TARGETS}
)

install(
  TARGETS
    spline_interpolation
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Install project namespaced headers
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
