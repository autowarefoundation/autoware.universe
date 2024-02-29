cmake_minimum_required(VERSION 3.5)
project(lidarslam)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  #add_compile_options(-Wall -Wextra -Wpedantic)
  #add_compile_options(-Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(scanmatcher REQUIRED)
find_package(graph_based_slam REQUIRED)
find_package(lidarslam_msgs REQUIRED)
find_package(ndt_omp_ros2 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenMP)
find_package(g2o REQUIRED
COMPONENTS
g2o::core
g2o::types_slam3d
g2o::types_slam3d_addons
g2o::solver_eigen
)

SET(CMAKE_CXX_FLAGS "-O2 -g ${CMAKE_CXX_FLAGS}")

if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)
link_directories(
  ${PCL_LIBRARY_DIRS}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(lidarslam
src/lidarslam.cpp
)

target_link_libraries(lidarslam
${PCL_LIBRARIES}
g2o::core
g2o::types_slam3d
g2o::types_slam3d_addons
g2o::solver_eigen
)

ament_target_dependencies(lidarslam
  rclcpp
  scanmatcher
  graph_based_slam
  lidarslam_msgs
  ndt_omp_ros2
  )

install(
  DIRECTORY launch param
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS 
  lidarslam
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()
