cmake_minimum_required(VERSION 3.14)
project(pose_estimator_manager)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(switch_rule
  SHARED
  src/switch_rule/map_based_rule.cpp
  src/switch_rule/map_based_rule_core.cpp
  src/rule_helper/eagleye_area.cpp
  src/rule_helper/ar_tag_position.cpp
  src/rule_helper/pcd_occupancy.cpp
)
target_include_directories(switch_rule PUBLIC include)

ament_auto_add_executable(${PROJECT_NAME}
  src/pose_estimator_manager_core.cpp
  src/pose_estimator_manager_node.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC include)
target_include_directories(${PROJECT_NAME} SYSTEM PRIVATE ${PCL_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} switch_rule)

# ==============================
function(add_testcase filepath)
  get_filename_component(filename ${filepath} NAME)
  string(REGEX REPLACE ".cpp" "" test_name ${filename})

  ament_add_gtest(${test_name} ${filepath})
  target_link_libraries("${test_name}" switch_rule)
  target_include_directories(${test_name} PUBLIC include)
  ament_target_dependencies(${test_name} ${${PROJECT_NAME}_FOUND_BUILD_DEPENDS})
endfunction()

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  add_testcase(test/map_based_rule.cpp)

  add_ros_test(
    test/pose_estimator_manager.test.py
    TIMEOUT "30"
  )
endif()

# ==============================
ament_auto_package(INSTALL_TO_SHARE
  launch
)
