cmake_minimum_required(VERSION 2.8.3)
project(emergency_handler)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  autoware_control_msgs
  autoware_system_msgs
  autoware_vehicle_msgs
  geometry_msgs
  roscpp
  std_msgs
)

catkin_package(
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
    autoware_control_msgs
    autoware_system_msgs
    autoware_vehicle_msgs
    geometry_msgs
    std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Target
## emergency_handler_node
add_executable(emergency_handler
  src/emergency_handler_node/main.cpp
  src/emergency_handler_node/emergency_handler_node.cpp
)

target_link_libraries(emergency_handler
  ${catkin_LIBRARIES}
)

add_dependencies(emergency_handler
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

# Install
## executables and libraries
install(
  TARGETS
    emergency_handler
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
