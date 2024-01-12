cmake_minimum_required(VERSION 3.5)
project(rviz_2d_overlay_msgs)

if (NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif ()
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
        "msg/OverlayText.msg"
        "msg/TrafficSignalArrayUI.msg"
        DEPENDENCIES
        std_msgs
        builtin_interfaces
        unique_identifier_msgs
        autoware_perception_msgs
)

ament_package()
