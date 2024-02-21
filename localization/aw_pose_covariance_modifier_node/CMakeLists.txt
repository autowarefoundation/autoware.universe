cmake_minimum_required(VERSION 3.14)
project(aw_pose_covariance_modifier_node)

find_package(autoware_cmake REQUIRED)
autoware_package()
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)

set(NODE_NAME ${PROJECT_NAME})
set(EXEC_NAME ${PROJECT_NAME}_exe)

add_executable(${PROJECT_NAME}  src/aw_pose_covariance_modifier_node.cpp)

ament_target_dependencies(${PROJECT_NAME} rclcpp geometry_msgs std_srvs)

install(TARGETS
        ${PROJECT_NAME}
        DESTINATION lib/${PROJECT_NAME}
)
include_directories(src/include/)

#ament_auto_add_library(${NODE_NAME}
#  src/include/aw_pose_covariance_modifier_node.hpp
#  src/aw_pose_covariance_modifier_node.cpp)

#rclcpp_components_register_node(${NODE_NAME}
#  PLUGIN "aw_pose_covariance_modifier_node::AWPoseCovarianceModifierNode"
#  EXECUTABLE ${EXEC_NAME})


ament_auto_package(INSTALL_TO_SHARE
  launch)
