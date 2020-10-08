cmake_minimum_required(VERSION 3.0.2)
project(freespace_planner)

add_compile_options(-std=c++14)

find_package(
  catkin REQUIRED COMPONENTS
    astar_search
    autoware_planning_msgs
    geometry_msgs
    nav_msgs
    roscpp
    tf2
    tf2_ros
    visualization_msgs
)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(freespace_planner
  src/freespace_planner/freespace_planner_node.cpp
  src/freespace_planner/main.cpp
)

target_link_libraries(freespace_planner
  ${catkin_LIBRARIES}
)

add_dependencies(freespace_planner
  ${catkin_EXPORTED_TARGETS}
)

install(
  TARGETS freespace_planner
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY
    launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
