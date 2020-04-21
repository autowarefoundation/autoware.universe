cmake_minimum_required(VERSION 2.8.3)

project(costmap_generator)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  autoware_perception_msgs
  autoware_planning_msgs
  grid_map_ros
  pcl_ros
  roscpp
  tf2
  tf2_ros
  lanelet2_extension
)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(costmap_generator_lib STATIC
  nodes/costmap_generator/points_to_costmap.cpp
  nodes/costmap_generator/objects_to_costmap.cpp
  nodes/costmap_generator/object_map_utils.cpp
)

target_link_libraries(costmap_generator_lib
  ${catkin_LIBRARIES}
)

add_dependencies(costmap_generator_lib
  ${catkin_EXPORTED_TARGETS}
)

add_executable(costmap_generator
  nodes/costmap_generator/costmap_generator_node.cpp
  nodes/costmap_generator/main.cpp
)

target_link_libraries(costmap_generator
  costmap_generator_lib
  ${catkin_LIBRARIES}
)

add_dependencies(costmap_generator
  ${catkin_EXPORTED_TARGETS}
)

install(
  TARGETS
    costmap_generator
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(
  DIRECTORY
    launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if (CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
endif()
