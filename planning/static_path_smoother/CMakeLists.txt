cmake_minimum_required(VERSION 3.14)
project(static_path_smoother)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

ament_auto_add_executable(static_path_smoother
  src/main.cpp
  src/functions.cpp
  src/optimization_node.cpp)

target_include_directories(static_path_smoother
  SYSTEM PUBLIC
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
)

target_link_libraries(static_path_smoother
  ${OpenCV_LIBRARIES}
)

ament_auto_package(
  INSTALL_TO_SHARE
    launch
    config
    rviz
)

install(PROGRAMS
  scripts/path_planning.sh
  DESTINATION lib/${PROJECT_NAME}
)
