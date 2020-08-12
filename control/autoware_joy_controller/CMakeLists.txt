cmake_minimum_required(VERSION 3.0.2)
project(autoware_joy_controller)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  autoware_control_msgs
  autoware_vehicle_msgs
  joy
  geometry_msgs
  roscpp
  sensor_msgs
  std_msgs
)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(autoware_joy_controller
  src/autoware_joy_controller/main.cpp
  src/autoware_joy_controller/autoware_joy_controller_node.cpp
)

target_link_libraries(autoware_joy_controller
  ${catkin_LIBRARIES}
)

add_dependencies(autoware_joy_controller
  ${catkin_EXPORTED_TARGETS}
)

# Install executables and/or libraries
install(
  TARGETS
    autoware_joy_controller
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install project namespaced headers
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

# Install directories
install(
  DIRECTORY
    launch
    config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
