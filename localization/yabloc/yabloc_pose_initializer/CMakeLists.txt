cmake_minimum_required(VERSION 3.5)
project(yabloc_pose_initializer)

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

# PCL
find_package(PCL REQUIRED COMPONENTS common kdtree)

# Sophus
find_package(Sophus REQUIRED)

# ===================================================
# Download DNN model
include(download.cmake)

# ===================================================
# Clear ${PYTHON_EXECUTABLE} defined by mrt_cmake_module
unset(PYTHON_EXECUTABLE)
message(STATUS "PYTHON_EXECUTABLE: ${PYTHON_EXECUTABLE}")

# ===================================================
# Service
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/Semseg.srv"
  DEPENDENCIES
  std_msgs
  sensor_msgs
)

# ===================================================
# Executable
# Camera
set(TARGET camera_pose_initializer_node)
ament_auto_add_executable(${TARGET}
  src/camera/lane_image.cpp
  src/camera/marker_module.cpp
  src/camera/projector_module.cpp
  src/camera/lanelet_util.cpp
  src/camera/camera_pose_initializer_core.cpp
  src/camera/camera_pose_initializer_node.cpp)
target_include_directories(${TARGET} PUBLIC include)
target_include_directories(${TARGET} SYSTEM PRIVATE ${EIGEN3_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
rosidl_target_interfaces(${TARGET} ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(${TARGET} ${PCL_LIBRARIES} Sophus::Sophus)

# GNSS
set(TARGET gnss_pose_initializer_node)
ament_auto_add_executable(${TARGET}
  src/gnss/gnss_pose_initializer_node.cpp)
target_include_directories(${TARGET} SYSTEM PRIVATE ${EIGEN3_INCLUDE_DIRS})

# Semantic segmentation
install(PROGRAMS
  src/semantic_segmentation/semseg_core.py
  src/semantic_segmentation/semseg_node.py
  src/semantic_segmentation/semseg_server.py
  DESTINATION lib/${PROJECT_NAME}
)

# ===================================================
ament_auto_package(INSTALL_TO_SHARE config data launch)