cmake_minimum_required(VERSION 2.8.3)
# set compiler
set(CMAKE_CXX_COMPILER "/usr/bin/clang++")

project(euclidean_cluster)
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
# 查找 OpenCV 包
find_package(OpenCV REQUIRED)



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
  # ${CURRENT_DIR}/ndt_cpu/include
  /usr/include/opencv4 #opencv的路径
  # ./include
  # $ENV{HOME}/dora/examples/c++-ros2-dataflow/build/ #C++ros的头文件路径
  #/home/crp/AutoDriving/dora_ndt_mapping/ndt_mapper/ndt_cpu/include
  $ENV{HOME}/dora/examples/c++-ros2-dataflow/build/ #C++ros的头文件路径
 
)

# link_libraries(/home/wzs/dora_project/dora-rs/dora/target/release)

# add_executable(${PROJECT_NAME} dora_node.cc dora_ndt_mapper.cc)
add_executable(euclidean_cluster src/main.cpp 
                        src/cluster.cpp
                        src/cluster.h
                        src/boxer.h
                        src/boxer.cpp
                        src/tracker.h
                        src/tracker.cpp
                        src/hungarian_bigraph_matcher.h
                        src/hungarian_bigraph_matcher.cc
                        src/tracker_object.h 
                        )
target_link_libraries(euclidean_cluster
  ${PCL_LIBRARIES}
  ${EIGEN_LIBRARIES}
  ${YAMLCPP_LIBRARIES}
  ${OpenCV_LIBS}# 链接 OpenCV 库
  # glog::glog
  $ENV{HOME}/dora/target/release/libdora_node_api_c.a
  # /home/crp/dora_project/dora-rs/dora-0.3.2/target/release/libdora_node_api_c.a
  # ${CURRENT_DIR}/ndt_cpu/build/libndt_cpu.a
  m
  rt
  dl 
  pthread
  pcap
)
