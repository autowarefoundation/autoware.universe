cmake_minimum_required(VERSION 3.0.2)
project(gyro_odometer)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf2
  tf2_ros
  tf2_geometry_msgs
  std_msgs
  geometry_msgs
  sensor_msgs
)

catkin_package(
  INCLUDE_DIRS include
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(gyro_odometer
        src/gyro_odometer_node.cpp
        src/gyro_odometer_core.cpp
)

add_dependencies(gyro_odometer ${${PROJECT_NAME}_EXPORTED_TARGETS}
                                  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(gyro_odometer ${catkin_LIBRARIES})

install(TARGETS
            gyro_odometer
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

install(
  DIRECTORY
    launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
