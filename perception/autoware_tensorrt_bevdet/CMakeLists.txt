cmake_minimum_required(VERSION 3.17)
project(autoware_tensorrt_bevdet)

add_compile_options(-W)
add_compile_options(-std=c++17)
set(CMAKE_CXX_FLAGS_RELEASE "-Wno-deprecated-declarations -O2")
set(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS} -std=c++17 -O0 -Xcompiler -fPIC -g -w -gencode=arch=compute_86,code=sm_86")
find_package(tensorrt_common)
if(NOT ${tensorrt_common_FOUND})
  message(WARNING "The tensorrt_common package is not found. Please check its dependencies.")
  return()
endif()

find_package(ament_cmake REQUIRED)
find_package(cudnn_cmake_module REQUIRED)
find_package(tensorrt_cmake_module REQUIRED)
find_package(rclcpp REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(autoware_perception_msgs REQUIRED)

find_package(CUDA REQUIRED)
find_package(CUDAToolkit)
find_package(CUDNN)
find_package(TENSORRT)
if(NOT (CUDAToolkit_FOUND AND CUDNN_FOUND AND TENSORRT_FOUND))
  message(WARNING "cuda, cudnn, tensorrt libraries are not found")
  return()
endif()

include_directories(
  include
  SYSTEM
  ${YAML_CPP_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${cv_bridge_INCLUDE_DIRS}
  ${tf2_geometry_msgs_INCLUDE_DIRS}
  ${autoware_perception_msgs_INCLUDE_DIRS}
)

cuda_add_executable(${PROJECT_NAME}_node
    src/bevdet_node.cpp
    src/bevdet.cpp
    src/preprocess.cu
    src/iou3d_nms.cu
    src/postprocess.cu
    src/data.cpp
    src/cpu_jpegdecoder.cpp
    src/nvjpegdecoder.cpp

    src/preprocess_plugin.cu
    src/bevpool_plugin.cu
    src/alignbev_plugin.cu
    src/gatherbev_plugin.cu
)

# Link libraries to executable
ament_target_dependencies(${PROJECT_NAME}_node
  "rclcpp"
  "sensor_msgs"
  "pcl_conversions"
  "cv_bridge"
  "autoware_perception_msgs"
  "tf2_geometry_msgs"
)

target_link_libraries(${PROJECT_NAME}_node
    yaml-cpp
    libnvinfer.so
    libnvonnxparser.so
    libz.so
    libjpeg.so
    rclcpp::rclcpp
    stdc++fs
    ${NVINFER}
    ${TENSORRT_LIBRARIES}
    ${CUDA_LIBRARIES}
    ${CUBLAS_LIBRARIES}
    ${CUDNN_LIBRARY}
    ${OpenCV_LIBS}
    ${PCL_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
