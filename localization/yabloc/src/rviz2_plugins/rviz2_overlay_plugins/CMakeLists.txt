cmake_minimum_required(VERSION 3.5)
project(rviz2_overlay_plugins)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED Core Widgets)
set(QT_LIBRARIES Qt5::Widgets)

set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
add_definitions(-DQT_NO_KEYWORDS)


set(HEADERS
  src/jsk_overlay_utils.hpp
  src/string_overlay_display.hpp
  src/image_overlay_display.hpp
)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/jsk_overlay_utils.cpp
  src/string_overlay_display.cpp
  src/image_overlay_display.cpp
  ${HEADERS}
)
target_link_libraries(${PROJECT_NAME} ${QT_LIBRARIES})

pluginlib_export_plugin_description_file(rviz_common plugins/plugin_description.xml)

ament_auto_package(INSTALL_TO_SHARE plugins)