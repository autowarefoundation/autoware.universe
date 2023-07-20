cmake_minimum_required(VERSION 3.14)
project(ar_tag_based_localizer)

find_package(autoware_cmake REQUIRED)
autoware_package()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(OpenCV REQUIRED)

#
# ar_tag_based_localizer
#
ament_auto_add_executable(ar_tag_based_localizer
  src/ar_tag_based_localizer_node.cpp
  src/ar_tag_based_localizer_core.cpp
  src/utils.cpp
)
target_include_directories(ar_tag_based_localizer
  SYSTEM PUBLIC
  ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(ar_tag_based_localizer ${OpenCV_LIBRARIES})

#
# tag_tf_caster
#
ament_auto_add_executable(tag_tf_caster
  src/tag_tf_caster_node.cpp
  src/tag_tf_caster_core.cpp
)
target_include_directories(tag_tf_caster
  SYSTEM PUBLIC
  ${OpenCV_INCLUDE_DIRS}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)

  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  launch
  config
)
