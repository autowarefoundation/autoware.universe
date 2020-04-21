cmake_minimum_required(VERSION 2.8.3)
project(lanelet2_extension)

find_package(PkgConfig)
find_path(GeographicLib_INCLUDE_DIR GeographicLib/Config.h
  PATH_SUFFIXES GeographicLib
  )
set(GeographicLib_INCLUDE_DIRS ${GeographicLib_INCLUDE_DIR})

find_library(GeographicLib_LIBRARIES
  NAMES Geographic
)

find_library(PUGIXML_LIBRARIES
  NAMES pugixml
)

find_path(PUGIXML_INCLUDE_DIRS
  NAMES pugixml.hpp
  PATH_SUFFIXES pugixml
)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  lanelet2_core
  lanelet2_io
  lanelet2_maps
  lanelet2_projection
  lanelet2_routing
  lanelet2_traffic_rules
  lanelet2_validation
  autoware_lanelet2_msgs
  geometry_msgs
  visualization_msgs
  roslint
)

set(ROSLINT_CPP_OPTS "--filter=-build/c++14")
roslint_cpp()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES lanelet2_extension_lib
  CATKIN_DEPENDS roscpp lanelet2_core lanelet2_io lanelet2_maps lanelet2_projection lanelet2_routing lanelet2_traffic_rules lanelet2_validation autoware_lanelet2_msgs geometry_msgs visualization_msgs
)

include_directories(
  include
  ${GeographicLib_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${PUGIXML_INCLUDE_DIRS}
)

add_definitions (${GeographicLib_DEFINITIONS})

add_library( lanelet2_extension_lib
  lib/autoware_osm_parser.cpp
  lib/autoware_traffic_light.cpp
  lib/detection_area.cpp
  lib/message_conversion.cpp
  lib/mgrs_projector.cpp
  lib/query.cpp
  lib/utilities.cpp
  lib/visualization.cpp
)

add_dependencies(lanelet2_extension_lib
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(lanelet2_extension_lib
  ${catkin_LIBRARIES}
  ${GeographicLib_LIBRARIES}
)

add_executable(lanelet2_extension_sample src/sample_code.cpp)
add_dependencies(lanelet2_extension_sample ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(lanelet2_extension_sample
  ${catkin_LIBRARIES}
  lanelet2_extension_lib
)

add_executable(autoware_lanelet2_validation src/validation.cpp)
add_dependencies(autoware_lanelet2_validation ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(autoware_lanelet2_validation
  ${catkin_LIBRARIES}
  ${PUGIXML_LIBRAREIS}
  lanelet2_extension_lib
)

install(TARGETS lanelet2_extension_lib lanelet2_extension_sample autoware_lanelet2_validation
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  roslint_add_test()
  find_package(rostest REQUIRED)
  add_rostest_gtest(message_conversion-test test/test_message_conversion.test test/src/test_message_conversion.cpp)
  target_link_libraries(message_conversion-test ${catkin_LIBRARIES} lanelet2_extension_lib)
  add_rostest_gtest(projector-test test/test_projector.test test/src/test_projector.cpp)
  target_link_libraries(projector-test ${catkin_LIBRARIES} lanelet2_extension_lib)
  add_rostest_gtest(query-test test/test_query.test test/src/test_query.cpp)
  target_link_libraries(query-test ${catkin_LIBRARIES} lanelet2_extension_lib)
  add_rostest_gtest(regulatory_elements-test test/test_regulatory_elements.test test/src/test_regulatory_elements.cpp)
  target_link_libraries(regulatory_elements-test ${catkin_LIBRARIES} lanelet2_extension_lib)
  add_rostest_gtest(utilities-test test/test_utilities.test test/src/test_utilities.cpp)
  target_link_libraries(utilities-test ${catkin_LIBRARIES} lanelet2_extension_lib)
endif()
