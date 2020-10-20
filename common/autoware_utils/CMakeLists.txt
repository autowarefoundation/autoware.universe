cmake_minimum_required(VERSION 3.5)
project(autoware_utils)

### Compile options
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
  add_compile_options(-Wno-unused-parameter)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(autoware_utils
  src/autoware_utils.cpp
)

# Test
if(AMENT_ENABLE_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_autoware_utils test/test_autoware_utils.test
    test/src/test_autoware_utils.cpp
    test/src/test_geometry.cpp
    test/src/test_normalization.cpp
    test/src/test_unit_conversion.cpp
  )
  target_link_libraries(test_autoware_utils
    autoware_utils
  )
endif ()

ament_auto_package()
