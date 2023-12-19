cmake_minimum_required(VERSION 3.14.4)
project(learned_model)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(Python3 COMPONENTS Interpreter Development)
find_package(pybind11 CONFIG)

ament_auto_add_library(${PROJECT_NAME} SHARED
  # include/sim_pymodel_steer_vel.hpp
  src/sim_pymodel_steer_vel.cpp
)
target_link_libraries(${PROJECT_NAME} pybind11::embed ${Python3_LIBRARIES})
target_include_directories(${PROJECT_NAME} PRIVATE ${Python3_INCLUDE_DIRS})

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

ament_auto_package()
