cmake_minimum_required(VERSION 3.0.2)
project(initial_pose_button_panel)

add_compile_options(-std=c++14)

set(CMAKE_BUILD_TYPE "Release")

find_package(catkin REQUIRED COMPONENTS
  rviz
  std_msgs
  geometry_msgs
  autoware_localization_srvs
)

if(rviz_QT_VERSION VERSION_LESS "5")
  message(STATUS "Using Qt4 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt4 ${rviz_QT_VERSION} EXACT REQUIRED QtCore QtGui)
  include(${QT_USE_FILE})
else()
  message(STATUS "Using Qt5 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED Core Widgets)
  set(QT_LIBRARIES Qt5::Widgets)
endif()

catkin_package(
  DEPENDS rviz
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
SET(CMAKE_CXX_FLAGS "-O2 -g -Wall -Wno-unused-result -DROS ${CMAKE_CXX_FLAGS}")


add_definitions(-DQT_NO_KEYWORDS -g)

set(CMAKE_AUTOMOC ON)
set(SOURCE_FILES
  src/initial_pose_button_panel.cpp
)

add_library(initial_pose_button_panel ${SOURCE_FILES} ${UIC_FILES})
target_link_libraries(initial_pose_button_panel ${QT_LIBRARIES} ${catkin_LIBRARIES} ${rviz_DEFAULT_PLUGIN_LIBRARIES})

install(TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY
    plugins
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
