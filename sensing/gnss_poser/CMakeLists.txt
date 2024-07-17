cmake_minimum_required(VERSION 2.8.3)
# set compiler
set(CMAKE_CXX_COMPILER "/usr/bin/clang++")

project(gnss_poser_core)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS "-std=c++14 -pthread -Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "-g")
# NOTE: CHOOSE THE O2/O3 is really important for speed! check more on optimization on compile
set(CMAKE_CXX_FLAGS_RELEASE "-O2")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")

set(CURRENT_DIR ${CMAKE_CURRENT_SOURCE_DIR})
message(STATUS "Current dir stored in variable: ${CURRENT_DIR}")

find_package(PkgConfig)
find_package(Eigen3 QUIET)
# find_package(GeographicLib REQUIRED)

# list(APPEND CMAKE_MODULE_PATH "/usr/share/cmake/geographiclib/FindGeographicLib.cmake")

pkg_check_modules(YAMLCPP REQUIRED yaml-cpp>=0.5)
if(NOT EIGEN3_FOUND)
  # Fallback to cmake_modules
  find_package(cmake_modules)
  find_package(Eigen)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
  set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES})  # Not strictly necessary as Eigen is head only
  # Possibly map additional variables to the EIGEN3_ prefix.
  message(STATUS "EIGEN_INCLUDE_DIRS:${EIGEN_INCLUDE_DIRS}")
else()
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
  message(STATUS "EIGEN3_INCLUDE_DIR:${EIGEN3_INCLUDE_DIR}")
endif()

# GeographicLib
set(GeoGraphic_DIR ${CMAKE_SOURCE_DIR}/third_party/GeographicLib)
include_directories(${GeoGraphic_DIR}/include)
link_directories(${GeoGraphic_DIR}/build/src)

include_directories(
  ${EIGEN3_INCLUDE_DIRS}
  ${YAMLCPP_INCLUDE_DIRS}
  ${GeographicLib_INCLUDE_DIRS}
  $ENV{HOME}/dora/apis/c/node/ #dora的头文件路径 node_api.h
  $ENV{HOME}/dora/apis/c/operator/
  # $ENV{HOME}/dora/examples/c++-ros2-dataflow/build/ #C++ros的头文件路径
  ../geo_pos_conv/include
  ./include
)

# add_library(gnss_poser_core src/gnss_poser_core.cc )# 添加所有需要的源文件
# target_link_libraries(gnss_poser_core
#   ${EIGEN_LIBRARIES}
#   ${YAMLCPP_LIBRARIES}
#   # glog::glog
#   $ENV{HOME}/dora/target/release/libdora_node_api_c.a
#   ../geo_pos_conv/build/libgeo_pos_conv.a
#   #$ENV{HOME}/dora/target/debug/libdora_node_api_c.a
#   m
#   rt
#   dl 
#   pthread
#   pcap
# )
add_executable(gnss_poser_core src/gnss_poser_core.cc 
#$ENV{HOME}/dora/examples/c++-ros2-dataflow/build/dora-ros2-bindings.cc 
#$ENV{HOME}/dora/examples/c++-ros2-dataflow/build/dora-node-api.cc
)
target_link_libraries(gnss_poser_core
  # ${PCL_LIBRARIES}
  ${EIGEN_LIBRARIES}
  ${YAMLCPP_LIBRARIES}
  #$ENV{HOME}/dora/target/debug/libdora_node_api_cxx.a
  $ENV{HOME}/dora/target/release/libdora_node_api_c.a
  # ../geo_pos_conv/build/libgeo_pos_conv.a
  #$ENV{HOME}/autoware.universe/sensing/geo_pos_conv/build/libgeo_pos_conv.a
  ${CMAKE_SOURCE_DIR}/../geo_pos_conv/build/libgeo_pos_conv.a
  libGeographicLib.so
  m
  rt
  dl 
  pthread
  pcap
)


# add_executable(gnss_poser_core_cxx src/gnss_poser_core_cxx.cc 
# $ENV{HOME}/dora/examples/c++-ros2-dataflow/build/dora-ros2-bindings.cc 
# $ENV{HOME}/dora/examples/c++-ros2-dataflow/build/dora-node-api.cc
# )
# target_link_libraries(gnss_poser_core_cxx
#   # ${PCL_LIBRARIES}
#   ${EIGEN_LIBRARIES}
#   ${YAMLCPP_LIBRARIES}
#   $ENV{HOME}/dora/target/debug/libdora_node_api_cxx.a
#   # $ENV{HOME}/dora/target/release/libdora_node_api_c.a
#   #../geo_pos_conv/build/libgeo_pos_conv.a
#   m
#   rt
#   dl 
#   pthread
#   pcap
# )
