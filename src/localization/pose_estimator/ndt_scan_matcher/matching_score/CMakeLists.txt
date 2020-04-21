cmake_minimum_required(VERSION 2.8.3)
project(matching_score)

add_compile_options(-std=c++14)

set(CMAKE_BUILD_TYPE "Release")

find_package(catkin REQUIRED)
find_package(PCL REQUIRED)

catkin_package()

include_directories(include ${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_library(matching_score src/matching_score.cpp)

target_link_libraries(matching_score ${catkin_LIBRARIES})

install(
  TARGETS
    matching_score
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
