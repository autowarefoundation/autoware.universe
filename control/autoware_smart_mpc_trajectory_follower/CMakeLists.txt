cmake_minimum_required(VERSION 3.14)
project(autoware_smart_mpc_trajectory_follower)

find_package(autoware_cmake REQUIRED)
find_package(python_cmake_module REQUIRED)
autoware_package()
ament_python_install_package(${PROJECT_NAME})
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
find_package(pybind11_vendor REQUIRED)
find_package(pybind11 REQUIRED)

execute_process(COMMAND bash -c "pip3 install numba==0.58.1 --force-reinstall")
execute_process(COMMAND bash -c "pip3 install GPy")

set(PYTHON_INSTALL_DIR "./scripts")

find_package(Eigen3 REQUIRED)
pybind11_add_module(proxima_calc SHARED
  src/proxima_calc.cpp
)
ament_target_dependencies(proxima_calc Eigen3)
install(TARGETS proxima_calc
  DESTINATION ${PYTHON_INSTALL_DIR}/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/pympc_trajectory_follower.py
  DESTINATION lib/${PROJECT_NAME}
)
ament_auto_package(
  INSTALL_TO_SHARE
)
