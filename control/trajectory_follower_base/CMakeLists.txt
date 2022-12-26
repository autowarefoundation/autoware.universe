#cmake_minimum_required(VERSION 3.14)
#project(trajectory_follower_base)
#
#find_package(ament_cmake REQUIRED)
#
#add_library(${PROJECT_NAME} INTERFACE)
#
#target_include_directories(${PROJECT_NAME} INTERFACE
#  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
#  "$<INSTALL_INTERFACE:include>"
#)
#
#install(TARGETS ${PROJECT_NAME}
#  EXPORT "export_${PROJECT_NAME}"
#  ARCHIVE DESTINATION lib
#  LIBRARY DESTINATION lib
#  RUNTIME DESTINATION lib
#  INCLUDES DESTINATION include
#)
#
#install(DIRECTORY include/${PROJECT_NAME}/
#  DESTINATION include/${PROJECT_NAME}
#)
#
##ament_export_libraries(${PROJECT_NAME})
#ament_export_targets("export_${PROJECT_NAME}")
#ament_package()

cmake_minimum_required(VERSION 3.14)
project(trajectory_follower_base)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/longitudinal_controller_base.cpp
  src/lateral_controller_base.cpp
)

ament_auto_package()
