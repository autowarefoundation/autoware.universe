cmake_minimum_required(VERSION 3.14)
project(pose_estimator_manager)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_executable(${PROJECT_NAME}
  src/pose_estimator_manager_core.cpp
  src/pose_estimator_manager_node.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC include)
target_include_directories(${PROJECT_NAME} SYSTEM PRIVATE ${PCL_INCLUDE_DIRS})

ament_auto_package(INSTALL_TO_SHARE
  launch
)
