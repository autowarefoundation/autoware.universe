cmake_minimum_required(VERSION 2.8.3)
project(autoware_error_monitor)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  autoware_system_msgs
  diagnostic_msgs
  roscpp
)

find_package(fmt REQUIRED)

catkin_package(
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
    autoware_system_msgs
    diagnostic_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Target
## autoware_error_monitor_node
add_executable(autoware_error_monitor
  src/autoware_error_monitor_node/main.cpp
  src/autoware_error_monitor_node/autoware_error_monitor_node.cpp
)

target_link_libraries(autoware_error_monitor
  fmt
  ${catkin_LIBRARIES}
)

add_dependencies(autoware_error_monitor
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

# Install
## executables and libraries
install(
  TARGETS
    autoware_error_monitor
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
