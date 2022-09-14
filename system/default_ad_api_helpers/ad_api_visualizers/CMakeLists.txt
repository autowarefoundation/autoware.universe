cmake_minimum_required(VERSION 3.14)
project(ad_api_visualizers)

find_package(autoware_cmake REQUIRED)
autoware_package()

install(PROGRAMS
  script/planning_factors.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_auto_package()
