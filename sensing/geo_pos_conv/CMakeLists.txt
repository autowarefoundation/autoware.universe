cmake_minimum_required(VERSION 3.0.2)
project(geo_pos_conv)

set(CMAKE_BUILD_TYPE "Release")

find_package(catkin REQUIRED COMPONENTS roscpp)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES geo_pos_conv
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(geo_pos_conv
  src/geo_pos_conv.cpp
)

target_link_libraries(geo_pos_conv
  ${catkin_LIBRARIES}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(TARGETS geo_pos_conv
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
