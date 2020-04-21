cmake_minimum_required(VERSION 2.8.3)
project(autoware_vehicle_rviz_plugin)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  rviz
  autoware_vehicle_msgs
)

catkin_package(
  LIBRARIES autoware_vehicle_rviz_plugin
  CATKIN_DEPENDS rviz autoware_vehicle_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
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

set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
add_definitions(-DQT_NO_KEYWORDS)

## Declare a C++ library
add_library(autoware_vehicle_rviz_plugin
  src/tools/turn_signal.cpp
  src/tools/console_meter.cpp
  src/tools/steering_angle.cpp
  src/tools/jsk_overlay_utils.cpp
  src/tools/velocity_history.cpp
)

target_link_libraries(autoware_vehicle_rviz_plugin ${QT_LIBRARIES} ${catkin_LIBRARIES})

install(TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY
    icons
    images
    plugins
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
