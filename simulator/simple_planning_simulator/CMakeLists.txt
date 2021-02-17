cmake_minimum_required(VERSION 3.5)
project(simple_planning_simulator)

### Compile options
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # add_compile_options(-Wall -Wextra -Wpedantic)
  add_compile_options(-Wno-unused-parameter)
endif()

### Dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(autoware_debug_msgs REQUIRED)
find_package(autoware_planning_msgs REQUIRED)
find_package(autoware_control_msgs REQUIRED)
find_package(autoware_vehicle_msgs REQUIRED)

# Add path of include dir
include_directories(include)

# Generate exe file
set(SIMPLE_PLANNING_SIMULATOR_SRC
  src/simple_planning_simulator_core.cpp
  src/vehicle_model/sim_model_interface.cpp
  src/vehicle_model/sim_model_ideal.cpp
  src/vehicle_model/sim_model_constant_acceleration.cpp
  src/vehicle_model/sim_model_time_delay.cpp
  src/vehicle_model/sim_model_util.cpp
)
add_executable(simple_planning_simulator_exe src/simple_planning_simulator_node.cpp ${SIMPLE_PLANNING_SIMULATOR_SRC})

# Add dependencies. use target_link_libaries() before Crystal
set(SIMPLE_PLANNING_SIMULATOR_DEPENDENCIES
  rclcpp
  std_msgs
  geometry_msgs
  tf2
  tf2_ros
  autoware_debug_msgs
  autoware_planning_msgs
  autoware_control_msgs
  autoware_vehicle_msgs
)
ament_target_dependencies(simple_planning_simulator_exe ${SIMPLE_PLANNING_SIMULATOR_DEPENDENCIES})


## Install executables and/or libraries
install(TARGETS simple_planning_simulator_exe
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

## Install project namespaced headers
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

# set at the end of cmakelists
ament_package()
