cmake_minimum_required(VERSION 3.0.2)
project(map_based_prediction)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  autoware_perception_msgs
  roscpp
  tf2_ros
  tf2_geometry_msgs
  lanelet2_extension
)

find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
    autoware_perception_msgs
    roscpp
    tf2_ros
    tf2_geometry_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(map_based_prediction
  src/map_based_prediction_main.cpp
  src/map_based_prediction_ros.cpp
  src/map_based_prediction.cpp
)

add_dependencies(map_based_prediction
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(map_based_prediction
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES}
)

install(TARGETS map_based_prediction
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY
    launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
