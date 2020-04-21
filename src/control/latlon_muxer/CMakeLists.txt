cmake_minimum_required(VERSION 2.8.3)
project(latlon_muxer)

add_compile_options(-std=c++14)

find_package(
  catkin REQUIRED COMPONENTS
    roscpp
    autoware_control_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    roscpp
    autoware_control_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(latlon_muxer src/node.cpp src/main.cpp)
target_link_libraries(latlon_muxer ${catkin_LIBRARIES})
add_dependencies(latlon_muxer ${catkin_EXPORTED_TARGETS})

install(
  TARGETS latlon_muxer
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
