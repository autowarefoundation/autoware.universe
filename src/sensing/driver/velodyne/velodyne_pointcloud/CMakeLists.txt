cmake_minimum_required(VERSION 2.8.3)
project(velodyne_pointcloud)

add_compile_options(-std=c++11)

set(CMAKE_BUILD_TYPE "Release")

set(${PROJECT_NAME}_CATKIN_DEPS
    angles
    nodelet
    pcl_ros
    roscpp
    roslib
    sensor_msgs
    tf
    tf2
    tf2_ros
    tf2_geometry_msgs
    velodyne_driver
    velodyne_msgs
    dynamic_reconfigure
    cv_bridge
    image_transport
)

find_package(catkin REQUIRED COMPONENTS
             ${${PROJECT_NAME}_CATKIN_DEPS} pcl_conversions)
find_package(Boost COMPONENTS signals)

# Resolve system dependency on yaml-cpp, which apparently does not
# provide a CMake find_package() module.
find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)
find_path(YAML_CPP_INCLUDE_DIR
          NAMES yaml_cpp.h
          PATHS ${YAML_CPP_INCLUDE_DIRS})
find_library(YAML_CPP_LIBRARY
             NAMES YAML_CPP
             PATHS ${YAML_CPP_LIBRARY_DIRS})

find_package(OpenCV REQUIRED)

link_directories(${YAML_CPP_LIBRARY_DIRS})

generate_dynamic_reconfigure_options(
  cfg/CloudNode.cfg cfg/TransformNode.cfg
)

if(NOT ${YAML_CPP_VERSION} VERSION_LESS "0.5")
  add_definitions(-DHAVE_NEW_YAMLCPP)
endif(NOT ${YAML_CPP_VERSION} VERSION_LESS "0.5")

include_directories(include ${catkin_INCLUDE_DIRS}
  ${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake
  ${OpenCV_INCLUDE_DIRS}
)

catkin_package(
    CATKIN_DEPENDS ${${PROJECT_NAME}_CATKIN_DEPS}
    INCLUDE_DIRS include
    LIBRARIES velodyne_rawdata)

#add_executable(dynamic_reconfigure_node src/dynamic_reconfigure_node.cpp)
#target_link_libraries(dynamic_reconfigure_node
#   ${catkin_LIBRARIES}
# )

add_subdirectory(src/lib)
add_subdirectory(src/conversions)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(FILES nodelets.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(PROGRAMS scripts/gen_calibration.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(
  DIRECTORY
    launch
    params
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if (CATKIN_ENABLE_TESTING)
  add_subdirectory(tests)
endif()
