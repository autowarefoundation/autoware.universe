cmake_minimum_required(VERSION 3.5)
project(lgsvl_interface)

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# Generate library
ament_auto_add_library(${PROJECT_NAME} SHARED
  src/lgsvl_interface.cpp
  src/lgsvl_interface_node.cpp)
autoware_set_compile_options(${PROJECT_NAME})

set(NODE_NAME ${PROJECT_NAME}_exe)
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "lgsvl_interface::LgsvlInterfaceNode"
  EXECUTABLE ${NODE_NAME}
)

if(BUILD_TESTING)
  # run linters
  # find_package(ament_lint_auto REQUIRED)
  # ament_lint_auto_find_test_dependencies()

  find_package(autoware_testing REQUIRED)
  add_smoke_test(${PROJECT_NAME} ${NODE_NAME})

  # gtest
  set(LGSVL_INTERFACE_GTEST test_${PROJECT_NAME}_gtest)

  ament_add_gtest(${LGSVL_INTERFACE_GTEST}
    test/src/gtest_main.cpp
    test/src/test_lgsvl_interface.cpp)
  autoware_set_compile_options(${LGSVL_INTERFACE_GTEST})
  target_include_directories(${LGSVL_INTERFACE_GTEST} PRIVATE "include" "test/include")
  target_link_libraries(${LGSVL_INTERFACE_GTEST} ${PROJECT_NAME})
  add_dependencies(${LGSVL_INTERFACE_GTEST} ${PROJECT_NAME})
  ament_target_dependencies(${LGSVL_INTERFACE_GTEST}
    "autoware_auto_common"
    "autoware_auto_vehicle_msgs"
    "lgsvl_msgs")
endif()

ament_auto_package(
  INSTALL_TO_SHARE
    param
    launch
)
