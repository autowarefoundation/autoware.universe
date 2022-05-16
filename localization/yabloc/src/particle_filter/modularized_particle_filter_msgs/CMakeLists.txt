cmake_minimum_required(VERSION 3.5)
project(modularized_particle_filter_msgs)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Particle.msg"
  "msg/ParticleArray.msg"
  DEPENDENCIES
    std_msgs
    geometry_msgs
)

ament_package()