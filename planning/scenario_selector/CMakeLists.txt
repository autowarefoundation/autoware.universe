cmake_minimum_required(VERSION 3.0.2)
project(scenario_selector)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  autoware_lanelet2_msgs
  autoware_planning_msgs
  lanelet2_extension
  roscpp
  tf2
  tf2_ros
  tf2_geometry_msgs
)

catkin_package(
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
    autoware_lanelet2_msgs
    autoware_planning_msgs
    lanelet2_extension
    tf2
    tf2_ros
    tf2_geometry_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Target
## scenario_selector_node
add_executable(scenario_selector
  src/scenario_selector_node/main.cpp
  src/scenario_selector_node/scenario_selector_node.cpp
)

target_link_libraries(scenario_selector
  ${catkin_LIBRARIES}
)

add_dependencies(scenario_selector
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

# Install
## executables and libraries
install(
  TARGETS
    scenario_selector
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## project namespaced headers
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

## launch files
install(
  DIRECTORY
    launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
