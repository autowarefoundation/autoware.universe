cmake_minimum_required(VERSION 3.5)
project(bayes_util)

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
find_package(Eigen3 REQUIRED)

find_package(Ceres REQUIRED)

# ===================================================
# Executable
set(TARGET bayes_util_lib)
ament_auto_add_library(${TARGET} SHARED
  src/bayes_util.cpp)
target_include_directories(${TARGET} PUBLIC include)
target_include_directories(${TARGET} SYSTEM PUBLIC ${EIGEN3_INCLUDE_DIRS} ${CERES_INCLUDE_DIRS})
target_link_libraries(${TARGET} ${CERES_LIBRARIES})

# ===================================================
# TEST
if(BUILD_TESTING)
  ament_add_gtest(
    test_debayes
    src/test_debayes.cpp
  )
  target_include_directories(test_debayes PRIVATE include)
  target_link_libraries(test_debayes bayes_util_lib)
endif()

# ===================================================
ament_auto_package()
