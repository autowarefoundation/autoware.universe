cmake_minimum_required(VERSION 3.5)
project(autoware_proj_version)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
ament_index_register_resource(autoware_metadata_packages CONTENT_FILE resource/version.json)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
