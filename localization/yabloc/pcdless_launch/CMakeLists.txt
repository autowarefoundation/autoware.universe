cmake_minimum_required(VERSION 3.5)

project(pcdless_launch)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

install(
  PROGRAMS
  scripts/extract_rosbag_for_loc.py
  scripts/play_rosbag_for_loc.py
  scripts/publish_control_command.py
  scripts/publish_video_file.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_auto_package(INSTALL_TO_SHARE launch config)
