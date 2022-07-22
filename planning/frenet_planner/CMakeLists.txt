cmake_minimum_required(VERSION 3.5)
project(frenet_planner)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

### Dependency for debug visualization executable
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

find_package(Qt5 COMPONENTS Widgets REQUIRED)
find_package(QCustomPlot REQUIRED)

ament_auto_add_library(frenet_planner SHARED
  DIRECTORY
  src/
)

target_link_libraries(frenet_planner
  Qt5::Widgets
  ${QCustomPlot_LIBRARIES}
)

# Debug exec
# add_executable(plot
#   src/frenet_planner/plot/plot_frenet.cpp
# )
# target_link_libraries(plot
#   frenet_planner
#   Qt5::Widgets
#   ${QCustomPlot_LIBRARIES}
# )


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  #ament_add_gtest(test_frenet_planner
  #  test/test_transform.cpp
  #  test/test_structures.cpp
  #)

  # target_link_libraries(test_frenet_planner
  #   frenet_planner
  # )
endif ()

ament_auto_package()
