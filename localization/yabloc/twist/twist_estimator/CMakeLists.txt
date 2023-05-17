cmake_minimum_required(VERSION 3.5)
project(twist_estimator)

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

# PCL
find_package(PCL REQUIRED)

# Sophus
find_package(Sophus REQUIRED)

# https://github.com/strasdat/Sophus/issues/388
# Sophus's macro cause compile error for the "-Wpedantic" flag, so make sure to import as a system.
# We must always call the following command after "ament_auto_find_build_dependencies()" because it rewrite CMAKE_NO_SYSTEM_FROM_IMPORTED to TRUE.
set(CMAKE_NO_SYSTEM_FROM_IMPORTED FALSE)

# ===================================================
# Executable
set(TARGET twist_estimator_node)
ament_auto_add_executable(${TARGET} src/twist_estimator_node.cpp src/twist_estimator_core.cpp)
target_include_directories(${TARGET} PUBLIC include)
target_link_libraries(${TARGET} Sophus::Sophus)

set(TARGET twist_visualizer_node)
ament_auto_add_executable(${TARGET} src/twist_visualizer_node.cpp src/twist_visualizer_core.cpp)
target_include_directories(${TARGET} PUBLIC include)
target_link_libraries(${TARGET} Sophus::Sophus)

# ===================================================
ament_auto_package(INSTALL_TO_SHARE config)