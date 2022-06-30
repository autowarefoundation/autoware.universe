cmake_minimum_required(VERSION 3.14)
project(localization_evaluator)

find_package(autoware_cmake REQUIRED)
autoware_package()

# find_package(Eigen3 REQUIRED)

# include_directories(
#   SYSTEM
#     ${EIGEN3_INCLUDE_DIR}
# )

ament_auto_add_executable(${PROJECT_NAME}
  src/localization_evaluator_node.cpp
  src/localization_evaluator_core.cpp
)

# ament_target_dependencies(localization_evaluator)

ament_auto_package(
  INSTALL_TO_SHARE
  launch
)
