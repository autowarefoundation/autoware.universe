cmake_minimum_required(VERSION 3.0.2)
project(awapi_awiv_adapter)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  message_runtime
  roscpp
  autoware_api_msgs
  autoware_system_msgs
  autoware_planning_msgs
  autoware_control_msgs
  autoware_vehicle_msgs
  diagnostic_msgs
  geometry_msgs
  pacmod_msgs
  std_msgs
  sensor_msgs
  tf2
  tf2_geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(awapi_awiv_adapter
  src/awapi_awiv_adapter_node.cpp
  src/awapi_awiv_adapter_core.cpp
  src/awapi_vehicle_state_publisher.cpp
  src/awapi_autoware_state_publisher.cpp
  src/awapi_lane_change_state_publisher.cpp
  src/awapi_obstacle_avoidance_state_publisher.cpp
  src/awapi_autoware_util.cpp
)

target_link_libraries(awapi_awiv_adapter
  ${catkin_LIBRARIES}
)

add_dependencies(awapi_awiv_adapter
  ${catkin_EXPORTED_TARGETS}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(
  DIRECTORY
    launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(TARGETS awapi_awiv_adapter
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
endif()
