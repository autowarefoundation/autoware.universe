cmake_minimum_required(VERSION 3.17)
project(cuda_api_wrappers)

include(CheckLanguage)
check_language(CUDA)
if(NOT CMAKE_CUDA_COMPILER)
  message(WARNING "cuda not found")
  return()
endif()

enable_language(CUDA)

include(FetchContent)
fetchcontent_declare(cuda-api-wrappers
  GIT_REPOSITORY https://github.com/eyalroz/cuda-api-wrappers.git
  GIT_TAG 831666a0bfd1af0f44f4fa234ee2d983d347fcaa # v0.6.1
)
fetchcontent_makeavailable(cuda-api-wrappers)

# Install package.xml (catkin/ament/rosdep)
install(FILES package.xml DESTINATION "share/${PROJECT_NAME}")

# Install cmake files
install(
  FILES
    ${cuda-api-wrappers_BINARY_DIR}/cuda-api-wrappers-config.cmake
    ${cuda-api-wrappers_BINARY_DIR}/cuda-api-wrappers-config-version.cmake
    ${cuda-api-wrappers_BINARY_DIR}/cuda-api-wrappers-targets.cmake
  DESTINATION
    "share/${PROJECT_NAME}/cmake"
)

# Allows colcon to find non-Ament packages when using workspace underlays
file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/share/ament_index/resource_index/packages/${PROJECT_NAME} "")
install(
  FILES
    ${CMAKE_CURRENT_BINARY_DIR}/share/ament_index/resource_index/packages/${PROJECT_NAME}
  DESTINATION
    share/ament_index/resource_index/packages
)
