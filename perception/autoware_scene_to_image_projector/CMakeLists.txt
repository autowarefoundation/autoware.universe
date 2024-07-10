cmake_minimum_required(VERSION 3.14)
project(autoware_scene_to_image_projector)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(ament_cmake_auto REQUIRED)

ament_auto_find_build_dependencies()

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
        ${OpenCV_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIRS}
)

ament_auto_add_library(${PROJECT_NAME}_node SHARED
        src/autoware_scene_to_image_projector_node.cpp
        )
target_link_libraries(${PROJECT_NAME}_node
        ${OpenCV_LIBRARIES}
        ${EIGEN3_LIBRARIES}
        )

rclcpp_components_register_node(${PROJECT_NAME}_node
        PLUGIN "autoware::scene_to_image_projector::SceneToImageProjectorNode"
        EXECUTABLE ${PROJECT_NAME}
        )

ament_auto_package(INSTALL_TO_SHARE
        launch
        config
        )