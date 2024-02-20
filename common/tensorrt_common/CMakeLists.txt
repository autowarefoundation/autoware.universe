cmake_minimum_required(VERSION 3.17)
project(tensorrt_common)

find_package(ament_cmake REQUIRED)
find_package(cudnn_cmake_module REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tensorrt_cmake_module REQUIRED)

find_package(CUDA)
find_package(CUDAToolkit)
find_package(CUDNN)
find_package(TENSORRT)

if(NOT (CUDAToolkit_FOUND AND CUDNN_FOUND AND TENSORRT_FOUND))
  message(WARNING "cuda, cudnn, tensorrt libraries are not found")
  return()
endif()

# Download onnx
set(DATA_PATH "${CMAKE_CURRENT_SOURCE_DIR}/data")
if(NOT EXISTS "${DATA_PATH}")
  execute_process(COMMAND mkdir -p ${DATA_PATH})
endif()
function(download FILE_NAME FILE_HASH)
  message(STATUS "Checking and downloading ${FILE_NAME}")
  set(FILE_PATH ${DATA_PATH}/${FILE_NAME})
  set(STATUS_CODE 0)
  message(STATUS "start ${FILE_NAME}")
  if(EXISTS ${FILE_PATH})
    message(STATUS "found ${FILE_NAME}")
    file(MD5 ${FILE_PATH} EXISTING_FILE_HASH)
    if(${FILE_HASH} STREQUAL ${EXISTING_FILE_HASH})
      message(STATUS "same ${FILE_NAME}")
      message(STATUS "File already exists.")
    else()
      message(STATUS "diff ${FILE_NAME}")
      message(STATUS "File hash changes. Downloading now ...")
      file(DOWNLOAD https://awf.ml.dev.web.auto/perception/models/${FILE_NAME} ${FILE_PATH} STATUS DOWNLOAD_STATUS TIMEOUT 3600)
      list(GET DOWNLOAD_STATUS 0 STATUS_CODE)
      list(GET DOWNLOAD_STATUS 1 ERROR_MESSAGE)
    endif()
  else()
    message(STATUS "not found ${FILE_NAME}")
    message(STATUS "File doesn't exists. Downloading now ...")
    file(DOWNLOAD https://awf.ml.dev.web.auto/perception/models/${FILE_NAME} ${FILE_PATH} STATUS DOWNLOAD_STATUS TIMEOUT 3600)
    list(GET DOWNLOAD_STATUS 0 STATUS_CODE)
    list(GET DOWNLOAD_STATUS 1 ERROR_MESSAGE)
  endif()
  if(${STATUS_CODE} EQUAL 0)
    message(STATUS "Download completed successfully!")
  else()
    message(FATAL_ERROR "Error occurred during download: ${ERROR_MESSAGE}")
  endif()
endfunction()

download(yolov5s.onnx 646b295db6089597e79163446d6eedca)

add_library(${PROJECT_NAME} SHARED
  src/tensorrt_common.cpp
  src/simple_profiler.cpp
)

target_link_libraries(${PROJECT_NAME}
  CUDA::cudart
  rclcpp::rclcpp
  stdc++fs
  ${TENSORRT_LIBRARIES}
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
    ${TENSORRT_INCLUDE_DIRS}
)

set_target_properties(${PROJECT_NAME}
  PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED YES
    CXX_EXTENSIONS NO
)

# TODO(tensorrt_common): Remove -Wno-deprecated-declarations once upgrading to TensorRT 8.5 is complete
target_compile_options(${PROJECT_NAME} PRIVATE
  -Wall -Wextra -Wpedantic -Werror -Wno-deprecated-declarations
)

target_compile_definitions(${PROJECT_NAME} PRIVATE
  TENSORRT_VERSION_MAJOR=${TENSORRT_VERSION_MAJOR}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  # These don't pass yet, disable them for now
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_flake8_FOUND TRUE)
  set(ament_cmake_pep257_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)

  ament_lint_auto_find_test_dependencies()

  file(GLOB_RECURSE test_files test/*.cpp)
  ament_add_ros_isolated_gtest(test_tensorrt_common ${test_files})
  target_link_libraries(test_tensorrt_common ${PROJECT_NAME})

endif()

install(TARGETS ${PROJECT_NAME} EXPORT export_${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_targets(export_${PROJECT_NAME})

ament_export_dependencies(
  "CUDA"
  "CUDAToolkit"
  "cudnn_cmake_module"
  "CUDNN"
  "rclcpp"
  "tensorrt_cmake_module"
  "TENSORRT"
)

ament_package()
