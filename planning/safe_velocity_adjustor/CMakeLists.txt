cmake_minimum_required(VERSION 3.5)
project(safe_velocity_adjustor)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(Boost REQUIRED)

ament_auto_add_library(safe_velocity_adjustor_node SHARED
  DIRECTORY src
)

rclcpp_components_register_node(safe_velocity_adjustor_node
  PLUGIN "safe_velocity_adjustor::SafeVelocityAdjustorNode"
  EXECUTABLE safe_velocity_adjustor
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_${PROJECT_NAME}
    test/test_collision_distance.cpp
  )
  target_link_libraries(test_${PROJECT_NAME}
    safe_velocity_adjustor_node
  )
endif()

ament_auto_package(
  INSTALL_TO_SHARE
    launch
    config
)
