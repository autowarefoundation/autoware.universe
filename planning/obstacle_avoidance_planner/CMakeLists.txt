cmake_minimum_required(VERSION 3.0.2)
project(obstacle_avoidance_planner)

find_package(catkin REQUIRED COMPONENTS
  autoware_planning_msgs
  autoware_perception_msgs
  autoware_system_msgs
  roscpp
  tf2_ros
  osqp_interface
  dynamic_reconfigure
)

generate_dynamic_reconfigure_options(
  cfg/AvoidancePlanner.cfg
)

catkin_package(
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
    autoware_planning_msgs
    autoware_perception_msgs
    autoware_system_msgs
    roscpp
    tf2_ros
    osqp_interface
)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_compile_options(-std=c++14)


add_executable(obstacle_avoidance_planner_node
  src/util.cpp
  src/debug.cpp
  src/spline_interpolate.cpp
  src/eb_path_optimizer.cpp
  src/process_cv.cpp
  src/node.cpp
  src/main.cpp)

add_dependencies(obstacle_avoidance_planner_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS})

target_include_directories(obstacle_avoidance_planner_node
  PUBLIC
    ${Opencv_INCLUDE_DIRS}
)

target_link_libraries(obstacle_avoidance_planner_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

install(
  TARGETS
    obstacle_avoidance_planner_node
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

install(
  DIRECTORY
    scripts
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)
