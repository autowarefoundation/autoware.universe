cmake_minimum_required(VERSION 3.5)
project(vmvl_trajectory)

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
# Executable
macro(ADD_EXE)
  cmake_parse_arguments(_ARG "" "TARGET" "SRC;LIB;INC" ${ARGN})

  set(SOURCE "")

  foreach(QUERY IN ITEMS ${_ARG_SRC})
    file(GLOB SOURCE_TMP ${QUERY})
    set(SOURCE "${SOURCE};${SOURCE_TMP}")
  endforeach()

  set(TARGET_NAME ${_ARG_TARGET})
  ament_auto_add_executable(${TARGET_NAME} ${SOURCE})
  target_include_directories(${TARGET_NAME} PUBLIC ${_ARG_INC})
  target_link_libraries(${TARGET_NAME} ${_ARG_LIB}  Sophus::Sophus)
endmacro()

ADD_EXE(TARGET fix_to_pose_node SRC src/fix_to_pose_node.cpp LIB ${PCL_LIBRARIES})
ADD_EXE(TARGET pose_to_path_node SRC src/pose_to_path_node.cpp)
ADD_EXE(TARGET twist_node SRC src/twist_estimator_*.cpp INC include)
ADD_EXE(TARGET pose_init_node SRC src/pose_initializer.cpp)
ADD_EXE(TARGET eagleye_bridge_node SRC src/eagleye_bridge.cpp)
ADD_EXE(TARGET velocity_report_node SRC src/velocity_report_node.cpp)

# ===================================================
ament_auto_package()
