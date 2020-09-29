cmake_minimum_required(VERSION 3.0.2)
project(shift_decider)

add_compile_options(-std=c++14)

find_package(
  catkin REQUIRED COMPONENTS
    autoware_control_msgs
    autoware_vehicle_msgs
    roscpp
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    autoware_control_msgs
    autoware_vehicle_msgs
    roscpp
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(shift_decider src/shift_decider.cpp src/main.cpp)
target_link_libraries(shift_decider ${catkin_LIBRARIES})
add_dependencies(shift_decider ${catkin_EXPORTED_TARGETS})

install(
  TARGETS shift_decider
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
