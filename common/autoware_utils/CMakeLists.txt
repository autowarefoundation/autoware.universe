cmake_minimum_required(VERSION 3.0.2)
project(autoware_utils)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  autoware_planning_msgs
  geometry_msgs
  tf2
  tf2_geometry_msgs
)

find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    autoware_utils
  CATKIN_DEPENDS
    autoware_planning_msgs
    geometry_msgs
    tf2
    tf2_geometry_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(autoware_utils
  src/autoware_utils.cpp
)

target_link_libraries(autoware_utils
  ${catkin_LIBRARIES}
)

add_dependencies(autoware_utils
  ${catkin_EXPORTED_TARGETS}
)

# Install executables and/or libraries
install(
  TARGETS
    autoware_utils
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install project namespaced headers
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

# Test
if (CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)

  add_rostest_gtest(test_autoware_utils test/test_autoware_utils.test
    test/src/test_autoware_utils.cpp
    test/src/test_geometry.cpp
    test/src/test_normalization.cpp
    test/src/test_unit_conversion.cpp
  )

  target_link_libraries(test_autoware_utils
    autoware_utils
    ${catkin_LIBRARIES}
  )
endif ()
