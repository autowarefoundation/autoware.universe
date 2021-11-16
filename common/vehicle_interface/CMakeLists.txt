cmake_minimum_required(VERSION 3.5)
project(vehicle_interface)

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# Library
ament_auto_add_library(${PROJECT_NAME} SHARED
  src/dbw_state_machine.cpp
  src/platform_interface.cpp
  src/safety_state_machine.cpp
  src/vehicle_interface_node.cpp)
autoware_set_compile_options(${PROJECT_NAME})

if(BUILD_TESTING)
  # Lint
  # find_package(ament_lint_auto REQUIRED)
  # ament_lint_auto_find_test_dependencies()
  # Test
  ament_add_gtest(${PROJECT_NAME}_test
    test/gtest_main.cpp
    test/error_handling.cpp
    test/filtering.cpp
    test/sanity_checks.cpp
    test/state_machine.hpp
    test/state_machine.cpp
    test/state_machine_bad_state.cpp
    test/state_machine_clamp.cpp
    test/state_machine_gear.cpp
    test/state_machine_headlight.cpp
    test/state_machine_node.cpp
    test/test_dbw_state_machine.cpp
    test/test_vi_node.hpp)
  autoware_set_compile_options(${PROJECT_NAME}_test)
  target_include_directories(${PROJECT_NAME}_test PRIVATE "include")
  target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME})
endif()

ament_auto_package(INSTALL_TO_SHARE param)
