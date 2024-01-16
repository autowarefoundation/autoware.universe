cmake_minimum_required(VERSION 3.14)
project(pose_estimator_arbiter)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(glog REQUIRED)

find_package(PCL REQUIRED COMPONENTS common)
include_directories(SYSTEM ${PCL_INCLUDE_DIRS})

ament_auto_add_library(switch_rule
  SHARED
  src/pose_estimator_arbiter/switch_rule/vector_map_based_rule.cpp
  src/pose_estimator_arbiter/rule_helper/pose_estimator_area.cpp

  # src/pose_estimator_arbiter/rule_helper/eagleye_area.cpp
  # src/pose_estimator_arbiter/rule_helper/ar_tag_position.cpp
  # src/pose_estimator_arbiter/rule_helper/pcd_occupancy.cpp
)
target_include_directories(switch_rule PUBLIC src)

ament_auto_add_executable(${PROJECT_NAME}
  src/pose_estimator_arbiter/pose_estimator_arbiter_core.cpp
  src/pose_estimator_arbiter/pose_estimator_arbiter_node.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC src)
target_link_libraries(${PROJECT_NAME} switch_rule glog::glog)

# ==============================
function(add_testcase filepath)
  get_filename_component(filename ${filepath} NAME)
  string(REGEX REPLACE ".cpp" "" test_name ${filename})

  ament_add_gtest(${test_name} ${filepath})
  target_link_libraries("${test_name}" switch_rule)
  target_include_directories(${test_name} PUBLIC src)
  ament_target_dependencies(${test_name} ${${PROJECT_NAME}_FOUND_BUILD_DEPENDS})
endfunction()

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  # TODO:
  # add_testcase(test/map_based_rule.cpp)
  # add_ros_test(
  # test/pose_estimator_arbiter.test.py
  # TIMEOUT "30"
  # )
endif()

# ==============================
ament_auto_package(INSTALL_TO_SHARE
  launch
)
