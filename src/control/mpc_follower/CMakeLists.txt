cmake_minimum_required(VERSION 2.8.3)
project(mpc_follower)

add_compile_options(-std=c++14 -Wall -Werror)

find_package(
  catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rostest
  rosunit
  std_msgs
  tf2
  tf2_ros
  autoware_planning_msgs
  autoware_control_msgs
  autoware_vehicle_msgs
  osqp_interface
  dynamic_reconfigure
)

generate_dynamic_reconfigure_options(cfg/MPCFollower.cfg)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    geometry_msgs
    roscpp
    std_msgs
    tf2
    tf2_ros
    autoware_planning_msgs
    autoware_control_msgs
    autoware_vehicle_msgs
    osqp_interface
)

SET(CMAKE_CXX_FLAGS "-O2 -g -Wall ${CMAKE_CXX_FLAGS}")

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

set(MPC_FOLLOWER_SRC
  src/mpc_utils.cpp
  src/interpolate.cpp
  src/mpc_trajectory.cpp
  src/lowpass_filter.cpp
  src/vehicle_model/vehicle_model_interface.cpp
  src/vehicle_model/vehicle_model_bicycle_kinematics.cpp
  src/vehicle_model/vehicle_model_bicycle_dynamics.cpp
  src/vehicle_model/vehicle_model_bicycle_kinematics_no_delay.cpp
  src/qp_solver/qp_solver_unconstr_fast.cpp
  src/qp_solver/qp_solver_osqp.cpp
)

add_executable(mpc_follower src/mpc_follower_node.cpp src/mpc_follower_core.cpp ${MPC_FOLLOWER_SRC})
add_dependencies(mpc_follower ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} mpc_follower_gencfg)
target_link_libraries(mpc_follower ${catkin_LIBRARIES})

install(
  TARGETS mpc_follower
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(
  PROGRAMS src/steer_simulator.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(
  DIRECTORY
    launch
    config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
