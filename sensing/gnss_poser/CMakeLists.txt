cmake_minimum_required(VERSION 3.0.2)
project(gnss_poser)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf2
  tf2_ros
  tf2_geometry_msgs
  std_msgs
  sensor_msgs
  geometry_msgs
  ublox_msgs
  geo_pos_conv
)

find_package(PkgConfig)
find_path(GeographicLib_INCLUDE_DIR GeographicLib/Config.h
  PATH_SUFFIXES GeographicLib
  )
set(GeographicLib_INCLUDE_DIRS ${GeographicLib_INCLUDE_DIR})
find_library(GeographicLib_LIBRARIES
  NAMES Geographic
)

catkin_package(
  INCLUDE_DIRS include
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GeographicLib_INCLUDE_DIRS}
)
link_directories(
  /usr/local/lib
)

add_executable(gnss_poser src/gnss_poser_node.cpp
                          src/gnss_poser_core.cpp)

add_dependencies(gnss_poser ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

 target_link_libraries(gnss_poser
   ${catkin_LIBRARIES}
   Geographic
 )

#############
## Install ##
#############

install(TARGETS gnss_poser
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY
    launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
