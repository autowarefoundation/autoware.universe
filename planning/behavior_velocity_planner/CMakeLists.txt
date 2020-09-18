cmake_minimum_required(VERSION 3.0.2)
project(behavior_velocity_planner)

# Compile as C++14, supported in ROS Melodic and newer
add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  autoware_perception_msgs
  autoware_planning_msgs
  autoware_perception_msgs
  geometry_msgs
  lanelet2_extension
  pcl_ros
  roscpp
  sensor_msgs
  tf2
  tf2_eigen
  tf2_geometry_msgs
  tf2_ros
  visualization_msgs
)

find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
    autoware_perception_msgs
    autoware_planning_msgs
    autoware_perception_msgs
    geometry_msgs
    lanelet2_extension
    pcl_ros
    sensor_msgs
    tf2
    tf2_eigen
    tf2_geometry_msgs
    tf2_ros
    visualization_msgs
)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

# Common
add_library(scene_module_lib STATIC
  src/utilization/path_utilization.cpp
  src/utilization/util.cpp
  src/utilization/interpolate.cpp
)

add_dependencies(scene_module_lib
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(scene_module_lib
  ${catkin_LIBRARIES}
)

# Scene Module: Stop Line
add_library(scene_module_stop_line STATIC
  src/scene_module/stop_line/manager.cpp
  src/scene_module/stop_line/scene.cpp
  src/scene_module/stop_line/debug.cpp
)

add_dependencies(scene_module_stop_line
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(scene_module_stop_line
  scene_module_lib
  ${catkin_LIBRARIES}
)

# Scene Module: Crosswalk
add_library(scene_module_crosswalk STATIC
  src/scene_module/crosswalk/manager.cpp
  src/scene_module/crosswalk/scene_crosswalk.cpp
  src/scene_module/crosswalk/scene_walkway.cpp
  src/scene_module/crosswalk/debug.cpp
  src/scene_module/crosswalk/util.cpp
)

add_dependencies(scene_module_crosswalk
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(scene_module_crosswalk
  scene_module_lib
  ${catkin_LIBRARIES}
)

# Scene Module: Intersection
add_library(scene_module_intersection STATIC
  src/scene_module/intersection/manager.cpp
  src/scene_module/intersection/scene_intersection.cpp
  src/scene_module/intersection/scene_merge_from_private_road.cpp
  src/scene_module/intersection/debug.cpp
  src/scene_module/intersection/util.cpp
)

add_dependencies(scene_module_intersection
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(scene_module_intersection
  scene_module_lib
  ${catkin_LIBRARIES}
)

# Scene Module: Traffic Light
add_library(scene_module_traffic_light STATIC
  src/scene_module/traffic_light/manager.cpp
  src/scene_module/traffic_light/scene.cpp
  src/scene_module/traffic_light/debug.cpp
)

add_dependencies(scene_module_traffic_light
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(scene_module_traffic_light
  scene_module_lib
  ${catkin_LIBRARIES}
)

# Scene Module: Blind Spot
add_library(scene_module_blind_spot STATIC
  src/scene_module/blind_spot/manager.cpp
  src/scene_module/blind_spot/scene.cpp
  src/scene_module/blind_spot/debug.cpp
)

add_dependencies(scene_module_blind_spot
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(scene_module_blind_spot
  scene_module_lib
  ${catkin_LIBRARIES}
)

# Scene Module: Detection Area
add_library(scene_module_detection_area STATIC
  src/scene_module/detection_area/manager.cpp
  src/scene_module/detection_area/scene.cpp
  src/scene_module/detection_area/debug.cpp
)

add_dependencies(scene_module_detection_area
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(scene_module_detection_area
  scene_module_lib
  ${catkin_LIBRARIES}
)

# Scene Module Manager
add_library(scene_module_manager STATIC
  src/planner_manager.cpp
)

add_dependencies(scene_module_manager
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(scene_module_manager
  scene_module_stop_line
  scene_module_crosswalk
  scene_module_intersection
  scene_module_traffic_light
  scene_module_blind_spot
  scene_module_detection_area
)

# Node
add_executable(behavior_velocity_planner_node
  src/node.cpp
  src/main.cpp
)

add_dependencies(behavior_velocity_planner_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(behavior_velocity_planner_node
  scene_module_manager
  ${catkin_LIBRARIES}
)

install(
  TARGETS
    behavior_velocity_planner_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY
    config
    launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
