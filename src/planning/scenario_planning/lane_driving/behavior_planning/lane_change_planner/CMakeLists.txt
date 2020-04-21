cmake_minimum_required(VERSION 2.8.3)
project(lane_change_planner)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  autoware_perception_msgs
  autoware_planning_msgs
  geometry_msgs
  lanelet2_extension
  roscpp
  sensor_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
)

find_package(OpenCV REQUIRED)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(lane_change_planner
  src/lane_change_planner_node.cpp
  src/lane_changer.cpp
  src/data_manager.cpp
	src/route_handler.cpp
  src/state_machine.cpp
  src/utilities.cpp

  src/state/following_lane.cpp
  src/state/forcing_lane_change.cpp
  src/state/executing_lane_change.cpp
  src/state/aborting_lane_change.cpp
  src/state/state_base_class.cpp
  src/state/blocked_by_obstacle.cpp
  src/state/common_functions.cpp
)
add_dependencies(lane_change_planner ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(lane_change_planner
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

install(
  TARGETS
    lane_change_planner
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
