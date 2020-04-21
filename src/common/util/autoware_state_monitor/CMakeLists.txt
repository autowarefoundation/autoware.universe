cmake_minimum_required(VERSION 2.8.3)
project(autoware_state_monitor)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  autoware_planning_msgs
  autoware_system_msgs
  roscpp
  topic_tools
  tf2
  tf2_ros
  tf2_geometry_msgs
)

find_package(fmt REQUIRED)

catkin_package(
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
    autoware_planning_msgs
    autoware_system_msgs
    topic_tools
    tf2
    tf2_ros
    tf2_geometry_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Target
## autoware_state_monitor_node
add_executable(autoware_state_monitor
  src/autoware_state_monitor_node/main.cpp
  src/autoware_state_monitor_node/autoware_state_monitor_node.cpp
  src/autoware_state_monitor_node/state_machine.cpp
)

target_link_libraries(autoware_state_monitor
  fmt
  ${catkin_LIBRARIES}
)

add_dependencies(autoware_state_monitor
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

# Install
## executables and libraries
install(
  TARGETS
    autoware_state_monitor
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## project namespaced headers
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

## directories
install(
  DIRECTORY
    launch
    config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
