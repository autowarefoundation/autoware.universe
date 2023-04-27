cmake_minimum_required(VERSION 3.5)
project(twist_converter)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

# ===================================================
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# ===================================================
# Eigen3
find_package(Eigen3 REQUIRED)

# ===================================================
# Executable
set(TARGET velocity_report_converter_node)
ament_auto_add_executable(${TARGET} src/velocity_report_node.cpp)

set(TARGET pose_twist_fuser_node)
ament_auto_add_executable(${TARGET} src/pose_twist_fuser_node.cpp)

# ===================================================
ament_auto_package()
