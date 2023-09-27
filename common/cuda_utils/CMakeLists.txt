cmake_minimum_required(VERSION 3.5)
project(cuda_utils)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(CUDA)

if(NOT ${CUDA_FOUND})
  message(WARNING "cuda is not found, so the cuda_utils package won't be built.")
  return()
endif()

include_directories(
  include
  ${CUDA_INCLUDE_DIRS}
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  file(GLOB_RECURSE test_files test/*.cpp)
  ament_add_ros_isolated_gtest(test_cuda_utils ${test_files})
  target_link_libraries(test_cuda_utils
    ${CUDA_LIBRARIES}
  )
endif()

ament_export_include_directories(include)
ament_export_dependencies(CUDA)

ament_auto_package()
