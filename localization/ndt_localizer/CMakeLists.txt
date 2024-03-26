cmake_minimum_required(VERSION 2.8.3)

project(ndt_localizer)

set(CMAKE_CXX_FLAGS "-std=c++14 -pthread -Wall -Wextra")
set(CMAKE_CXX_COMPILER "/usr/bin/clang++")

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS_DEBUG "-g")

set(CMAKE_CXX_FLAGS_RELEASE "-O2")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")

find_package(PCL REQUIRED)
find_package(Eigen3 QUIET)
find_package(PkgConfig)
find_package(Threads REQUIRED)
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

include_directories(${CMAKE_SOURCE_DIR}/include)

include_directories(
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${YAMLCPP_INCLUDE_DIRS}
  $ENV{HOME}/dora/apis/c/node #dora的头文件路径 node_api.h
  $ENV{HOME}/dora/apis/c/operator
  #/home/char/dora_/dora-rs/dora
)

add_executable(${PROJECT_NAME} src/map_loader.cpp src/ndt_localizer.cpp src/points_downsampler.cpp dora_node.cc)

target_link_libraries(${PROJECT_NAME}
  ${PCL_LIBRARIES}
  ${EIGEN_LIBRARIES}
  ${YAMLCPP_LIBRARIES}
  $ENV{HOME}/dora/target/release/libdora_node_api_c.a
  #/usr/local/lib/libyaml-cpp.a
  rt
)




