cmake_minimum_required(VERSION 2.8.3)
project(mission_planner)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  autoware_planning_msgs
  geometry_msgs
  lanelet2_extension
  roscpp
  tf2_ros
  tf2_geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS autoware_planning_msgs lanelet2_extension roscpp tf2_ros
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(mission_planner
  lib/mission_planner_base.cpp
  src/mission_planner_lanelet2/mission_planner_lanelet2.cpp
  src/mission_planner_lanelet2/route_handler.cpp
  src/mission_planner_lanelet2/mission_planner_main.cpp
  src/mission_planner_lanelet2/utility_functions.cpp
)
add_dependencies(mission_planner
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(mission_planner
  ${catkin_LIBRARIES}
)

install(TARGETS mission_planner
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY
    launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
