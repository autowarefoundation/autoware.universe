cmake_minimum_required(VERSION 3.5)
project(vmvl_map)

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

# ===================================================
# Library
file(GLOB REGULATORY_ELEMENT_SOURCE 3rd/regulatory_elements/lib/*cpp)
ament_auto_add_library(ll2_util SHARED
  src/ll2_util.cpp
  ${REGULATORY_ELEMENT_SOURCE})
target_include_directories(ll2_util PUBLIC include 3rd/regulatory_elements/include)

# ===================================================
# Executable
set(TARGET ground_server_node)
ament_auto_add_executable(${TARGET}
  src/ground_server_core.cpp
  src/ground_server_node.cpp)
target_include_directories(${TARGET} PUBLIC include)
target_include_directories(${TARGET} SYSTEM PRIVATE ${EIGEN3_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
target_link_libraries(${TARGET} ${PCL_LIBRARIES} Sophus::Sophus)

set(TARGET ll2_decompose_node)
ament_auto_add_executable(${TARGET}
  src/ll2_decompose_core.cpp
  src/ll2_decompose_node.cpp)
target_include_directories(${TARGET} PUBLIC include)
target_include_directories(${TARGET} SYSTEM PRIVATE ${EIGEN3_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
target_link_libraries(${TARGET} ${PCL_LIBRARIES} ll2_util Sophus::Sophus)

# ===================================================
ament_auto_package()
