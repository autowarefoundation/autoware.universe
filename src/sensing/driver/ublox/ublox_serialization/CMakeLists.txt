cmake_minimum_required(VERSION 2.8.3)

project(ublox_serialization)

find_package(catkin REQUIRED COMPONENTS roscpp roscpp_serialization)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp roscpp_serialization)

install(DIRECTORY include/
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
)
