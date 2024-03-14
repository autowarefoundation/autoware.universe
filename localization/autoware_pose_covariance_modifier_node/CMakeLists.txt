cmake_minimum_required(VERSION 3.14)
project(autoware_pose_covariance_modifier_node)

find_package(autoware_cmake REQUIRED)
autoware_package()
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
set(NODE_NAME ${PROJECT_NAME})
set(EXEC_NAME ${PROJECT_NAME}_exe)

add_executable(${PROJECT_NAME}  src/autoware_pose_covariance_modifier_node.cpp)

ament_target_dependencies(${PROJECT_NAME} rclcpp geometry_msgs rcl_interfaces)

install(TARGETS
        ${PROJECT_NAME}
        DESTINATION lib/${PROJECT_NAME}
)
include_directories(src/include/)

ament_auto_package(INSTALL_TO_SHARE
        config
        launch)
