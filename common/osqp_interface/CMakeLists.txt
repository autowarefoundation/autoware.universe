cmake_minimum_required(VERSION 3.0.2)
project(osqp_interface)

add_compile_options(-std=c++14 -Ofast)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rostest
  rosunit
)

find_package(Eigen3 REQUIRED)

find_package(osqp REQUIRED)
get_target_property(osqp_INCLUDE_DIR osqp::osqpstatic INTERFACE_INCLUDE_DIRECTORIES)

catkin_package(
  INCLUDE_DIRS
    include
    ${osqp_INCLUDE_DIR}
  LIBRARIES
    osqp_interface
)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${osqp_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
)

add_library(osqp_interface src/osqp_interface.cpp src/csc_matrix_conv.cpp)

target_link_libraries(osqp_interface
  osqp::osqpstatic
  ${catkin_LIBRARIES}
)

install(
  TARGETS
    osqp_interface
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Install project namespaced headers
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
