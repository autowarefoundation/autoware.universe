cmake_minimum_required(VERSION 3.5)
project(autoware_debug_tools)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

#ament_python_install_package(${PROJECT_NAME} PACKAGE_DIR "scripts/")

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

install(PROGRAMS scripts/stop_reason2pose.py scripts/pose2tf.py scripts/tf2pose.py
                scripts/case_converter.py scripts/self_pose_listener.py
                scripts/stop_reason2tf DESTINATION lib/${PROJECT_NAME})

install(FILES DESTINATION share/${PROJECT_NAME})
