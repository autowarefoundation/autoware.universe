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
find_package(OpenMP)
if(OpenMP_FOUND)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/tensorrt_yolox.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  OpenCV
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
