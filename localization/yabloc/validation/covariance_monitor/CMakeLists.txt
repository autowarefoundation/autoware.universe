cmake_minimum_required(VERSION 3.5)
project(covariance_monitor)

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

# Sophus
find_package(Sophus REQUIRED)

# ===================================================
# Executable
set(TARGET covariance_monitor_node)
ament_auto_add_executable(${TARGET} src/covariance_node.cpp)
target_include_directories(${TARGET} PUBLIC include ${EIGEN_INCLUDE_DIRS})
target_link_libraries(${TARGET} Sophus::Sophus)

# ===================================================
ament_auto_package()
