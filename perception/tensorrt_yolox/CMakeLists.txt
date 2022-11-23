cmake_minimum_required(VERSION 3.5)
project(tensorrt_yolox)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
find_package(OpenCV REQUIRED)
find_package(CUDA REQUIRED)
find_package(CUDNN REQUIRED)
find_package(TENSORRT REQUIRED)
if(NOT (${CUDA_FOUND} AND ${CUDNN_FOUND} AND ${TENSORRT_FOUND}))
  message(WARNING "cuda, cudnn, tensorrt libraries are not found")
  return()
endif()

find_package(OpenMP)
if(OpenMP_FOUND)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

# The following libraries, which are named cuda_utils and tensorrt_common,
# are planned to be independent packages
##########
# [lib]　cuda_utils
install(
  DIRECTORY lib/include/cuda_utils/
  DESTINATION include/cuda_utils
)

# ament_export_include_directories(lib/include)
ament_export_dependencies(CUDA)

##########
# [lib] tensorrt_common
cuda_add_library(tensorrt_common SHARED
  lib/src/tensorrt_common/tensorrt_common.cpp
)

ament_target_dependencies(tensorrt_common
  rclcpp
)

target_include_directories(tensorrt_common
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/lib/include>
  $<INSTALL_INTERFACE:include>
  ${TENSORRT_INCLUDE_DIRS}
)

target_link_libraries(tensorrt_common
  ${TENSORRT_LIBRARIES}
  stdc++fs
)

target_compile_definitions(tensorrt_common PRIVATE
  TENSORRT_VERSION_MAJOR=${TENSORRT_VERSION_MAJOR}
)

list(APPEND tensorrt_common_LIBRARIES "tensorrt_common")

# ament_export_include_directories(lib/include)
ament_export_dependencies(cudnn_cmake_module)
ament_export_dependencies(CUDNN)
ament_export_dependencies(tensorrt_cmake_module)
ament_export_dependencies(TENSORRT)

include_directories(
  lib/include
)

##########
# tensorrt_yolox
ament_auto_add_library(${PROJECT_NAME} SHARED
  src/tensorrt_yolox.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  OpenCV
)

target_link_libraries(${PROJECT_NAME}
  ${tensorrt_common_LIBRARIES}
)

target_compile_definitions(${PROJECT_NAME} PRIVATE
  TENSORRT_VERSION_MAJOR=${TENSORRT_VERSION_MAJOR}
)

ament_auto_add_library(yolox_single_image_inferece_node SHARED
  src/yolox_single_image_inference_node.cpp
)

ament_target_dependencies(yolox_single_image_inferece_node
  OpenCV
)

target_link_libraries(yolox_single_image_inferece_node
  ${PROJECT_NAME}
  stdc++fs
)

target_compile_definitions(yolox_single_image_inferece_node PRIVATE
  TENSORRT_VERSION_MAJOR=${TENSORRT_VERSION_MAJOR}
)

rclcpp_components_register_node(yolox_single_image_inferece_node
  PLUGIN "tensorrt_yolox::YoloXSingleImageInferenceNode"
  EXECUTABLE yolox_single_image_inferece
)

ament_auto_add_library(${PROJECT_NAME}_node SHARED
  src/tensorrt_yolox_node.cpp
)

ament_target_dependencies(${PROJECT_NAME}_node
  OpenCV
)

target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}
)

target_compile_definitions(${PROJECT_NAME}_node PRIVATE
  TENSORRT_VERSION_MAJOR=${TENSORRT_VERSION_MAJOR}
)

rclcpp_components_register_node(${PROJECT_NAME}_node
  PLUGIN "tensorrt_yolox::TrtYoloXNode"
  EXECUTABLE ${PROJECT_NAME}_node_exe
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
