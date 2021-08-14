cmake_minimum_required(VERSION 3.5)
project(simple_planning_simulator)

### Compile options
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

### Dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

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
ament_auto_add_executable(simple_planning_simulator_exe src/simple_planning_simulator_node.cpp ${SIMPLE_PLANNING_SIMULATOR_SRC})

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

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# set at the end of cmakelists
ament_auto_package()
