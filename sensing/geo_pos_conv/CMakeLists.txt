cmake_minimum_required(VERSION 2.8.3)
# set compiler
set(CMAKE_CXX_COMPILER "/usr/bin/clang++")

project(geo_pos_conv)
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


include_directories(
  ${EIGEN3_INCLUDE_DIRS}
  $ENV{HOME}/dora_project/dora/apis/c/node #dora的头文件路径 node_api.h
  $ENV{HOME}/dora_project/dora/apis/c/operator
  ./include
)

add_library(geo_pos_conv src/geo_pos_conv.cc )# 添加所有需要的源文件
target_link_libraries(geo_pos_conv
  ${EIGEN_LIBRARIES}
  # glog::glog
  $ENV{HOME}/dora_project/dora/target/release/libdora_node_api_c.a
  #$ENV{HOME}/dora/target/debug/libdora_node_api_c.a
  m
  rt
  dl 
  pthread
  pcap
)
