cmake_minimum_required(VERSION 2.8.3)
# set compiler
set(CMAKE_CXX_COMPILER "/usr/bin/clang++")

project(test_c_node)
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



pkg_check_modules(YAMLCPP REQUIRED yaml-cpp>=0.5)

 
include_directories(
  ${YAMLCPP_INCLUDE_DIRS}
  $ENV{HOME}/dora/apis/c/node #dora的头文件路径 node_api.h
  #/home/nvidia/dora_project/dora/apis/c/node
  $ENV{HOME}/dora/apis/c/operator
  $ENV{HOME}/dora/examples/c++-ros2-dataflow/build/ #C++ros的头文件路径
  
)

add_executable(node_A node_A.cc )
target_link_libraries(node_A
  ${YAMLCPP_LIBRARIES}
  # glog::glog
  $ENV{HOME}/dora/target/release/libdora_node_api_c.a
  m
  rt
  dl 
  pthread
)
add_executable(node_B node_B.cc )
target_link_libraries(node_B
  ${YAMLCPP_LIBRARIES}
  # glog::glog
  $ENV{HOME}/dora/target/release/libdora_node_api_c.a
  m
  rt
  dl 
  pthread
)
add_executable(node_C node_C.cpp)
target_link_libraries(node_C
  ${YAMLCPP_LIBRARIES}
  # glog::glog
  $ENV{HOME}/dora/target/release/libdora_node_api_c.a
  m
  rt
  dl 
  pthread
)
 
