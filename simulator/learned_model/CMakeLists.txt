cmake_minimum_required(VERSION 3.14.4)
project(learned_model)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()



## dependencies
find_package(ament_cmake_auto REQUIRED)
# ament_auto_find_build_dependencies()

# find_package(autoware_cmake REQUIRED)
# autoware_package()

find_package(Python3 COMPONENTS Interpreter Development)
find_package(pybind11 CONFIG)

message("Pybind11_FOUND:${pybind11_FOUND}")
message("Pybind11_Version:${pybind11_VERSION}")

message("Python_FOUND:${Python3_FOUND}")
message("Python_VERSION:${Python3_VERSION}")
message("Python_Development_FOUND:${Python3_Development_FOUND}")
message("Python_LIBRARIES:${Python3_LIBRARIES}")

include_directories(include)

# Component

# ament_auto_add_library
add_library(${PROJECT_NAME} SHARED
  # .cpp and .hpp files to instal
  include/sim_pymodel_steer_vel.hpp
)


target_include_directories(${PROJECT_NAME}
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")



install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}/
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

set_target_properties(${PROJECT_NAME} PROPERTIES LINKER_LANGUAGE CXX)
target_compile_options(${PROJECT_NAME} PRIVATE -fvisibility=hidden)

ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(Python3 pybind11)


message("CMAKE_INSTALL_PREFIX  ${CMAKE_INSTALL_PREFIX}")

target_link_libraries(${PROJECT_NAME} pybind11::embed ${Python3_LIBRARIES})

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})


if(BUILD_TESTING)
  
endif()



ament_auto_package()
