cmake_minimum_required(VERSION 3.5)
project(refine_optimizer)

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

# OpenCV
find_package(OpenCV REQUIRED)

# PCL
find_package(PCL REQUIRED)

# Eigen3
find_package(Eigen3 REQUIRED)

# ceres
find_package(Ceres REQUIRED)

# ===================================================
# Executable
ament_auto_add_executable(refine_node
  src/optimizer.cpp
  src/config.cpp
  src/refine_core.cpp
  src/refine_node.cpp
)
target_include_directories(refine_node PUBLIC include
  ${PCL_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(refine_node ${PCL_LIBRARIES} ${OpenCV_LIBS} ${CERES_LIBRARIES})
target_compile_definitions(refine_node PUBLIC SOPHUS_USE_BASIC_LOGGING=1)

# ===================================================
ament_auto_package(INSTALL_TO_SHARE)