cmake_minimum_required(VERSION 3.5)
project(shape_estimation)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
  add_compile_options(-Wno-unused-parameter)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(PCL REQUIRED COMPONENTS common)
find_package(pcl_conversions REQUIRED)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)


set(SHAPE_ESTIMATION_SOURCES
  src/node.cpp
  src/shape_estimator.cpp
  src/model/normal/bounding_box.cpp
  src/model/normal/convex_hull.cpp
  src/model/normal/cylinder.cpp
  src/model/yaw_fixed/bounding_box.cpp
  src/filter/car_filter.cpp
  src/filter/bus_filter.cpp
  src/filter/truck_filter.cpp
  src/filter/no_filter.cpp
  src/corrector/normal/car_corrector.cpp
  src/corrector/normal/bus_corrector.cpp
  src/corrector/normal/truck_corrector.cpp
  src/corrector/normal/no_corrector.cpp
  src/corrector/yaw_fixed/car_corrector.cpp
  src/corrector/yaw_fixed/bus_corrector.cpp
  src/corrector/yaw_fixed/truck_corrector.cpp
  src/corrector/yaw_fixed/no_corrector.cpp
)

set(SHAPE_ESTIMATION_HEADERS
  src/model/normal/bounding_box.hpp
  src/model/normal/convex_hull.hpp
  src/model/normal/cylinder.hpp
  src/model/yaw_fixed/bounding_box.hpp
  src/filter/car_filter.hpp
  src/filter/bus_filter.hpp
  src/filter/truck_filter.hpp
  src/filter/no_filter.hpp
  src/corrector/normal/car_corrector.hpp
  src/corrector/normal/bus_corrector.hpp
  src/corrector/normal/truck_corrector.hpp
  src/corrector/normal/no_corrector.hpp
  src/corrector/yaw_fixed/car_corrector.hpp
  src/corrector/yaw_fixed/bus_corrector.hpp
  src/corrector/yaw_fixed/truck_corrector.hpp
  src/corrector/yaw_fixed/no_corrector.hpp
)

ament_auto_add_library(shape_estimation_node SHARED
  ${SHAPE_ESTIMATION_SOURCES}
  ${SHAPE_ESTIMATION_HEADERS}
)

set(SHAPE_ESTIMATION_DEPENDENCIES
  rclcpp
  autoware_perception_msgs
  tf2
  tf2_geometry_msgs
  tf2_eigen
  PCL
  pcl_conversions
  OpenCV
  Eigen3
)
ament_target_dependencies(shape_estimation_node ${SHAPE_ESTIMATION_DEPENDENCIES})
target_include_directories(shape_estimation_node
    PUBLIC
        $<INSTALL_INTERFACE:include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
)

rclcpp_components_register_node(shape_estimation_node
  PLUGIN "ShapeEstimationNode"
  EXECUTABLE shape_estimation
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(INSTALL_TO_SHARE
  launch
)
