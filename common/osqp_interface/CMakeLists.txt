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

find_package(Eigen3 REQUIRED)

find_package(osqp_vendor REQUIRED)

find_package(osqp REQUIRED)
get_target_property(OSQP_INCLUDE_DIR osqp::osqpstatic INTERFACE_INCLUDE_DIRECTORIES)

set(EXTERNAL_INCLUDE_DIRS
  "${EIGEN3_INCLUDE_DIR}"
  "${OSQP_INCLUDE_DIR}"
  )

ament_auto_add_library(osqp_interface
  src/osqp_interface.cpp
  src/csc_matrix_conv.cpp
  include/osqp_interface/osqp_interface.h
  include/osqp_interface/csc_matrix_conv.h
  )
target_link_libraries(osqp_interface osqp::osqpstatic)

target_include_directories(osqp_interface PUBLIC "${EXTERNAL_INCLUDE_DIRS}")
# needed so clients of this package don't need to worry about includes of this package
ament_export_include_directories("${EXTERNAL_INCLUDE_DIRS}")

ament_auto_package()
