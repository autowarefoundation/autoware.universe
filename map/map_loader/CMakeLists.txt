cmake_minimum_required(VERSION 3.5)
project(map_loader)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()

find_package(PCL REQUIRED COMPONENTS io)
find_package(Boost REQUIRED COMPONENTS filesystem)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_executable(pointcloud_map_loader src/pointcloud_map_loader/main.cpp src/pointcloud_map_loader/pointcloud_map_loader_node.cpp)
target_link_libraries(pointcloud_map_loader ${Boost_LIBRARIES} ${PCL_LIBRARIES})

ament_auto_add_executable(lanelet2_map_loader src/lanelet2_map_loader/lanelet2_map_loader.cpp)

ament_auto_add_executable(lanelet2_map_visualization src/lanelet2_map_loader/lanelet2_map_visualization.cpp)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(INSTALL_TO_SHARE
  launch
)
