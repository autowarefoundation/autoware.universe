cmake_minimum_required(VERSION 2.8.3)
project(pose_history)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rviz
  geometry_msgs
)

find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED Core Widgets)
set(QT_LIBRARIES Qt5::Widgets)
set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
add_definitions(-DQT_NO_KEYWORDS)

catkin_package(
  CATKIN_DEPENDS roscpp rviz
)

include_directories(
  src
  ${catkin_INCLUDE_DIRS}
  ${Qt5_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/pose_history_display.hpp
  src/pose_history_display.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES} ${QT_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(
  DIRECTORY
    icons
    plugins
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
