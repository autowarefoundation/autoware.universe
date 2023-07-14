cmake_minimum_required(VERSION 3.8)
project(tracking_object_merger)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

include_directories(
  SYSTEM
    ${EIGEN3_INCLUDE_DIR}
)

ament_auto_add_library(mu_successive_shortest_path SHARED
  src/tracking_object_merger/data_association/mu_successive_shortest_path/mu_successive_shortest_path_wrapper.cpp
)

# ament_auto_add_library(tracking_object_merger SHARED
#   src/tracking_object_merger/data_association/data_association.cpp
#   src/tracking_object_merger/node.cpp
# )

# target_link_libraries(tracking_object_merger
#   mu_successive_shortest_path
#   Eigen3::Eigen
# )

# rclcpp_components_register_node(tracking_object_merger
#   PLUGIN "object_association::ObjectAssociationMergerNode"
#   EXECUTABLE tracking_object_merger_node
# )

ament_auto_package(INSTALL_TO_SHARE
  launch
  config
)

ament_package()
