cmake_minimum_required(VERSION 2.8.3)
project(tamagawa_imu_driver)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  can_msgs
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(tag_serial_driver src/tag_serial_driver.cpp)
target_link_libraries(tag_serial_driver ${catkin_LIBRARIES})

add_executable(tag_can_driver src/tag_can_driver.cpp)
target_link_libraries(tag_can_driver ${catkin_LIBRARIES})

install(TARGETS
tag_serial_driver
DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY
    launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
