cmake_minimum_required(VERSION 3.14)
project(map_projection_loader)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(PCL REQUIRED COMPONENTS io)

ament_auto_add_library(map_projection_loader SHARED
  src/map_projection_loader_node.cpp
)
target_link_libraries(map_projection_loader yaml-cpp)

# ament_auto_find_build_dependencies()

# ament_auto_add_library(map_projection_loader_lib SHARED
#   src/map_projection_loader.cpp
# )

# target_link_libraries(map_projection_loader_lib yaml-cpp)

# ament_auto_add_executable(map_projection_loader src/map_projection_loader_node.cpp)

# target_compile_options(map_projection_loader PUBLIC -g -Wall -Wextra -Wpedantic -Werror)

# target_link_libraries(map_projection_loader map_projection_loader_lib)
# target_include_directories(map_projection_loader PUBLIC include)

# function(add_testcase filepath)
#   get_filename_component(filename ${filepath} NAME)
#   string(REGEX REPLACE ".cpp" "" test_name ${filename})

#   ament_add_gtest(${test_name} ${filepath})
#   target_link_libraries("${test_name}" map_projection_loader_lib)
#   ament_target_dependencies(${test_name} ${${PROJECT_NAME}_FOUND_BUILD_DEPENDS})
# endfunction()

if(BUILD_TESTING)
  add_launch_test(
    test/test_map_projection_loader_launch.test.py
    TIMEOUT "30"
  )
  install(DIRECTORY
    test/data/
    DESTINATION share/${PROJECT_NAME}/test/data/
  )
  # find_package(ament_cmake_gtest REQUIRED)

  # set(TEST_FILES
  # test/test_map_projection_loader.cpp)

  # foreach(filepath ${TEST_FILES})
  #   add_testcase(${filepath})
  # endforeach()
endif()


ament_auto_package(INSTALL_TO_SHARE
  launch
)
