cmake_minimum_required(VERSION 3.0.2)
project(autoware_planning_rviz_plugin)
add_compile_options(-std=c++14 -Wall)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rviz
  autoware_planning_msgs
)

find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED Core Widgets)
set(QT_LIBRARIES Qt5::Widgets)
set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
add_definitions(-DQT_NO_KEYWORDS)
find_package(Eigen3 REQUIRED)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    rviz
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_library(autoware_planning_rviz_plugin
  include/path/display.hpp
  src/path/display.cpp
  include/trajectory/display.hpp
  src/trajectory/display.cpp
  include/mission_checkpoint/mission_checkpoint.hpp
  src/mission_checkpoint/mission_checkpoint.cpp
  src/tools/jsk_overlay_utils.cpp
  src/tools/max_velocity.cpp
)

target_link_libraries(autoware_planning_rviz_plugin
  ${catkin_LIBRARIES} ${QT_LIBRARIES}
)

add_dependencies(autoware_planning_rviz_plugin
  ${catkin_EXPORTED_TARGETS}
)

install(TARGETS autoware_planning_rviz_plugin
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)


install(
  DIRECTORY
    icons
    plugins
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
