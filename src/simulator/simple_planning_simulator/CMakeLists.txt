cmake_minimum_required(VERSION 2.8.3)
project(simple_planning_simulator)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  tf2
  tf2_ros
  geometry_msgs
  autoware_planning_msgs
  autoware_control_msgs
  autoware_vehicle_msgs
  rostest
  rosunit
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    roscpp
    std_msgs
    tf2
    tf2_ros
    geometry_msgs
    autoware_planning_msgs
    autoware_control_msgs
    autoware_vehicle_msgs
)

###########
## Build ##
###########

SET(CMAKE_CXX_FLAGS "-O2 -g -Wall ${CMAKE_CXX_FLAGS}")

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

set(simple_planning_simulator_SRC
  src/simple_planning_simulator_core.cpp
  src/vehicle_model/sim_model_interface.cpp
  src/vehicle_model/sim_model_ideal.cpp
  src/vehicle_model/sim_model_constant_acceleration.cpp
  src/vehicle_model/sim_model_time_delay.cpp
)
add_executable(simple_planning_simulator src/simple_planning_simulator_node.cpp ${simple_planning_simulator_SRC})
target_link_libraries(simple_planning_simulator ${catkin_LIBRARIES})
add_dependencies(simple_planning_simulator ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Install executables and/or libraries
install(TARGETS simple_planning_simulator
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Install project namespaced headers
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(
  DIRECTORY
    launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_install_python(PROGRAMS scripts/fitParamDelayInputModel.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

if (CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
endif ()
