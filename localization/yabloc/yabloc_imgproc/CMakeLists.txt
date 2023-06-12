cmake_minimum_required(VERSION 3.5)
project(yabloc_imgproc)

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

# PCL
find_package(PCL REQUIRED COMPONENTS common)

# Sophus
find_package(Sophus REQUIRED)

# ===================================================
# Executable
# line segment detection
set(TARGET lsd_node)
ament_auto_add_executable(${TARGET}
  src/lsd/lsd_node.cpp
  src/lsd/lsd_core.cpp)
target_include_directories(${TARGET} PUBLIC include)
target_include_directories(${TARGET} SYSTEM PUBLIC ${EIGEN3_INCLUDE_DIRS})
target_link_libraries(${TARGET} ${OpenCV_LIBS})

# graph based segmentation
set(TARGET graph_segment_node)
ament_auto_add_executable(${TARGET}
  src/graph_segment/graph_segment_node.cpp
  src/graph_segment/graph_segment_core
  src/graph_segment/similar_area_searcher.cpp)
target_include_directories(${TARGET} PUBLIC include)
target_include_directories(${TARGET} SYSTEM PUBLIC ${EIGEN3_INCLUDE_DIRS})
target_link_libraries(${TARGET} ${OpenCV_LIBS})

# segment filter
set(TARGET segment_filter_node)
ament_auto_add_executable(${TARGET}
  src/segment_filter/segment_filter_node.cpp
  src/segment_filter/segment_filter_core.cpp)
target_include_directories(${TARGET} PUBLIC include ${EIGEN3_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
target_link_libraries(${TARGET} ${PCL_LIBRARIES} ${OpenCV_LIBS})

# undistort
set(TARGET undistort_node)
ament_auto_add_executable(${TARGET}
  src/undistort/undistort_node.cpp)
target_link_libraries(${TARGET} ${OpenCV_LIBS})

# line_segments_overlay
set(TARGET line_segments_overlay_node)
ament_auto_add_executable(${TARGET}
  src/line_segments_overlay/line_segments_overlay_core.cpp
  src/line_segments_overlay/line_segments_overlay_node.cpp)
target_include_directories(${TARGET} PUBLIC include ${EIGEN_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
target_link_libraries(${TARGET} Sophus::Sophus ${PCL_LIBRARIES})

# lanelet2_overlay
set(TARGET lanelet2_overlay_node)
ament_auto_add_executable(${TARGET}
  src/lanelet2_overlay/lanelet2_overlay_core.cpp
  src/lanelet2_overlay/lanelet2_overlay_node.cpp)
target_include_directories(${TARGET} PUBLIC include ${EIGEN_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
target_link_libraries(${TARGET} Sophus::Sophus ${PCL_LIBRARIES})

# ===================================================
ament_auto_package(INSTALL_TO_SHARE config launch)
