cmake_minimum_required(VERSION 3.5)
project(graph_segment)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # add_compile_options(-Wall -Wextra -Wpedantic -Werror)
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ===================================================
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# ===================================================
# Eigen3
find_package(Eigen3 REQUIRED)

# OpenCV
find_package(OpenCV REQUIRED)

# ===================================================
# Executable
set(TARGET graph_segment_node)
ament_auto_add_executable(${TARGET} src/graph_segment_node.cpp src/graph_segment_core src/similar_area_searcher.cpp)
target_include_directories(${TARGET} PUBLIC include)
target_include_directories(${TARGET} SYSTEM PUBLIC ${EIGEN3_INCLUDE_DIRS})
target_link_libraries(${TARGET} ${OpenCV_LIBS})

# ===================================================
ament_auto_package(INSTALL_TO_SHARE config)