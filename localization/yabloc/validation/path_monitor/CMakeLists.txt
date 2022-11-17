cmake_minimum_required(VERSION 3.5)
project(path_monitor)

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
find_package(PCL REQUIRED COMPONENTS common)

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
  target_include_directories(${TARGET_NAME} SYSTEM PUBLIC ${EIGEN_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
  target_link_libraries(${TARGET_NAME} ${_ARG_LIB})
endmacro()

ADD_EXE(TARGET fix_to_pose SRC src/fix_to_pose_node.cpp INC include)
ADD_EXE(TARGET pose_to_path SRC src/pose_to_path_node.cpp INC include)

# ===================================================
ament_auto_package()
