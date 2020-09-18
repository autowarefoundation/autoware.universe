cmake_minimum_required(VERSION 3.0.2)
project(naive_path_prediction)

find_package(catkin REQUIRED COMPONENTS
  autoware_perception_msgs
  geometry_msgs
  tf2
  roscpp
)

catkin_package()

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(naive_path_prediction_node src/node.cpp)

add_dependencies(naive_path_prediction_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(naive_path_prediction_node
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

install(
  DIRECTORY
    launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
