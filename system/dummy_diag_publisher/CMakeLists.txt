cmake_minimum_required(VERSION 2.8.3)
project(dummy_diag_publisher)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  diagnostic_updater
  dynamic_reconfigure
  roscpp
)

find_package(fmt REQUIRED)

generate_dynamic_reconfigure_options(
  cfg/DummyDiagPublisher.cfg
)

catkin_package(
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
    diagnostic_updater
    dynamic_reconfigure
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Target
## dummy_diag_publisher_node
add_executable(dummy_diag_publisher
  src/dummy_diag_publisher_node/main.cpp
  src/dummy_diag_publisher_node/dummy_diag_publisher_node.cpp
)

target_link_libraries(dummy_diag_publisher
  fmt
  ${catkin_LIBRARIES}
)

add_dependencies(dummy_diag_publisher
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

# Install
## executables and libraries
install(
  TARGETS
    dummy_diag_publisher
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## project namespaced headers
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

## directories
install(
  DIRECTORY
    launch
    config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
