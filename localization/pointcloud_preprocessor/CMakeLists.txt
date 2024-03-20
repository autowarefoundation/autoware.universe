cmake_minimum_required(VERSION 2.8.3)
# set compiler
set(CMAKE_CXX_COMPILER "/usr/bin/clang++")

project(lidar)
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
find_package(PCL REQUIRED)
find_package(Eigen3 QUIET)
find_package(PkgConfig)



pkg_check_modules(YAMLCPP REQUIRED yaml-cpp>=0.5)

if(NOT EIGEN3_FOUND)
  # Fallback to cmake_modules
  find_package(cmake_modules REQUIRED)
  find_package(Eigen REQUIRED)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
  set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES})  # Not strictly necessary as Eigen is head only
  # Possibly map additional variables to the EIGEN3_ prefix.
else()
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

message(${EIGEN_INCLUDE_DIRS})

include_directories(
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${YAMLCPP_INCLUDE_DIRS}
  $ENV{HOME}/dora/apis/c/node #dora的头文件路径 node_api.h
  $ENV{HOME}/dora/apis/c/operator
  ${CURRENT_DIR}/rs_driver/src #雷达的头文件路径
  $ENV{HOME}/dora/examples/c++-ros2-dataflow/build/ #C++ros的头文件路径
  
)

add_executable(ring_outlier_fliter ring_outlier_fliter.cc )
target_link_libraries(ring_outlier_fliter
  ${PCL_LIBRARIES}
  ${EIGEN_LIBRARIES}
  ${YAMLCPP_LIBRARIES}
  # glog::glog
  $ENV{HOME}/dora/target/release/libdora_node_api_c.a
  m
  rt
  dl 
  pthread
  pcap
)


add_executable(crop_box_point crop_box_point.cc )
target_link_libraries(crop_box_point
  ${PCL_LIBRARIES}
  ${EIGEN_LIBRARIES}
  ${YAMLCPP_LIBRARIES}
  # glog::glog
  $ENV{HOME}/dora/target/release/libdora_node_api_c.a
  m
  rt
  dl 
  pthread
  pcap
)

add_executable(voxel_grid_downsample_filtre voxel_grid_downsample_filtre.cc)
target_link_libraries(voxel_grid_downsample_filtre
  ${PCL_LIBRARIES}
  ${EIGEN_LIBRARIES}
  ${YAMLCPP_LIBRARIES}
  $ENV{HOME}/dora/target/release/libdora_node_api_c.a
  #$ENV{HOME}/dora_project/dora-rs/dora/target/debug/libdora_node_api_cxx.a
  m
  rt
  dl 
  pthread
  pcap
)



