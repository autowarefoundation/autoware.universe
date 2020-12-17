cmake_minimum_required(VERSION 3.9)
project(ndt_omp)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
  add_compile_options(-Wno-unused-parameter)
endif()

set(CMAKE_BUILD_TYPE "Release")

find_package(ament_cmake REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters kdtree registration io)
find_package(OpenMP)

###########
## Build ##
###########

add_library(ndt_omp
  src/ndt_omp/voxel_grid_covariance_omp.cpp
  src/ndt_omp/ndt_omp.cpp
)

target_include_directories(ndt_omp
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include> 
)

if (CMAKE_SYSTEM_PROCESSOR MATCHES "(x86)|(X86)|(amd64)|(AMD64)")
  if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    target_compile_options(ndt_omp
      PRIVATE
        # -mavx causes a lot of errors!!
        -msse -msse2 -msse3 -msse4 -msse4.1 -msse4.2
    )
  endif()
endif()

set(NDT_OMP_DEPENDENCIES
  PCL
  OpenMP
)


# Naively, I'd want to do something like:
# ament_target_dependencies(ndt_omp PUBLIC ${NDT_OMP_DEPENDENCIES})
# But there are several problems with that. For one, you'll have a hundred
# errors like "undefined reference to `pcl::something::Something`". That's
# because it only links `libpcl_common.so` for whatever reason, but we also
# want `libpcl_io.so` etc. So we do this PCL thing manually:
target_compile_definitions(ndt_omp PUBLIC ${PCL_DEFINITIONS})
target_include_directories(ndt_omp PUBLIC ${PCL_INCLUDE_DIRS})
target_link_libraries(ndt_omp PUBLIC ${PCL_LIBRARIES})
target_link_directories(ndt_omp PUBLIC ${PCL_LIBRARY_DIRS})

# As for OpenMP, the recommended format is the following, which is not
# supported by `ament_target_dependencies()` either.
if(OpenMP_CXX_FOUND)
  target_link_libraries(ndt_omp PUBLIC OpenMP::OpenMP_CXX)
else()
  message(WARNING "OpenMP not found")
endif()
# See https://cliutils.gitlab.io/modern-cmake/chapters/packages/OpenMP.html
# but it doesn't matter anyway because at the time of writing, omp.h is in
# /usr/lib/gcc/x86_64-linux-gnu/9/include/ which is one of the default
# include directories, and libgomp.so is in /usr/lib/x86_64-linux-gnu/,
# which is one of the default linker search paths.

ament_export_targets(export_ndt_omp HAS_LIBRARY_TARGET)
ament_export_dependencies(${NDT_OMP_DEPENDENCIES})


# Same story for the executable
add_executable(align
  apps/align.cpp
)

target_compile_definitions(align PRIVATE ${PCL_DEFINITIONS})
target_include_directories(align PRIVATE ${PCL_INCLUDE_DIRS})
target_link_libraries(align PRIVATE ${PCL_LIBRARIES})
target_link_directories(align PRIVATE ${PCL_LIBRARY_DIRS})
if(OpenMP_CXX_FOUND)
  target_link_libraries(align PRIVATE OpenMP::OpenMP_CXX)
else()
  message(WARNING "OpenMP not found")
endif()

# Again, no chance for `ament_target_dependencies()` since you can't depend on
# a local library target: "ament_target_dependencies() the passed package name
# 'ndt_omp' was not found" – so we need to add ndt_omp with
# `target_link_libraries(align PUBLIC ndt_omp)` instead.
target_link_libraries(align PRIVATE ndt_omp)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS ndt_omp
  EXPORT export_ndt_omp
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
