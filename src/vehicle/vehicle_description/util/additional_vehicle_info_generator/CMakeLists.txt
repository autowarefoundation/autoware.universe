cmake_minimum_required(VERSION 2.8.3)
project(additional_vehicle_info_generator)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(additional_vehicle_info_generator
  src/additional_vehicle_info_generator_node.cpp
)

add_dependencies(additional_vehicle_info_generator
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(additional_vehicle_info_generator
  ${catkin_LIBRARIES}
)

install(TARGETS additional_vehicle_info_generator
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY
    launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
