cmake_minimum_required(VERSION 2.8.3)
project(pose2twist)

add_compile_options(-std=c++14)

set(CMAKE_BUILD_TYPE "Release")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf2_geometry_msgs
  std_msgs
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(pose2twist
  src/pose2twist_node.cpp
  src/pose2twist_core.cpp
)

add_dependencies(pose2twist
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(pose2twist ${catkin_LIBRARIES})

install(
  TARGETS
    pose2twist
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY
    launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
