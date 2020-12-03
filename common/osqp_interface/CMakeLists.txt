cmake_minimum_required(VERSION 3.5)
project(osqp_interface)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Ofast)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

find_package(osqp_vendor REQUIRED)

find_package(osqp REQUIRED)
get_target_property(OSQP_INCLUDE_DIR osqp::osqpstatic INTERFACE_INCLUDE_DIRECTORIES)

ament_auto_add_library(osqp_interface STATIC
  src/osqp_interface.cpp
  src/csc_matrix_conv.cpp
  include/osqp_interface/osqp_interface.hpp
  include/osqp_interface/csc_matrix_conv.hpp
  )

target_include_directories(osqp_interface PUBLIC "${OSQP_INCLUDE_DIR}")
ament_target_dependencies(osqp_interface Eigen3)
ament_target_dependencies(osqp_interface osqp_vendor)

# crucial so downstream package builds because osqp_interface exposes osqp.hpp
ament_export_include_directories("${OSQP_INCLUDE_DIR}")
# crucial so the linking order is correct in a downstream package: libosqp_interface.a should come before libosqp.a
ament_export_libraries(osqp::osqpstatic)

ament_auto_package()
