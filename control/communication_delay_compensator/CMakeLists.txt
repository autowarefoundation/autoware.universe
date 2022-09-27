cmake_minimum_required(VERSION 3.10)
project(communication_delay_compensator)

# require that dependencies from package.xml be available
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies(REQUIRED
  ${${PROJECT_NAME}_BUILD_DEPENDS}
  ${${PROJECT_NAME}_BUILDTOOL_DEPENDS})


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(
    -Wall
    -Wextra
    -Wpedantic
    -Wno-unused-parameter
    -Wno-unused-variable
    -Wno-conversion
    -Wno-sign-conversion
    -Wdouble-promotion
    # -Wfloat-equal
    -Waddress
    -fvisibility=default) # visibility hidden or default is set here.
endif()

## Release, Debug, RelWithDebInfo, MinSizeRel
#set(CMAKE_BUILD_TYPE "Release")
#
#if (CMAKE_BUILD_TYPE MATCHES RELEASE)
#    set(CMAKE_CXX_FLAGS_RELEASE "-O3")
#    message("In release mode ...")
#endif ()

## ------------------ External Library Dependencies ------------------------
# For Eigen vectorization.
include(CheckCXXCompilerFlag)

check_cxx_compiler_flag("-march=native" COMPILER_SUPPORTS_MARCH_NATIVE)
if(COMPILER_SUPPORTS_MARCH_NATIVE)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native")
  message(STATUS "Enabling MARCH NATIVE ")
endif()


## ----------------- Library headers and sources -------------------------------
set(CDOB_LIB_INCLUDE_DIRS
  include)

set(CDOB_LIB delay_compensation_lib)

set(CDOB_LIB_HEADERS
  include/communication_delay_compensator_core.hpp
  include/utils_delay_observer/delay_compensation_utils.hpp
  include/utils_delay_observer/delay_compensation_utils.hpp
  include/vehicle_models/vehicle_definitions.hpp
  include/vehicle_models/vehicle_kinematic_error_model.hpp)

set(CDOB_LIB_SOURCE
  src/communication_delay_compensator_core.cpp
  src/utils_delay_observer/delay_compensation_utils.cpp
  src/utils_delay_observer/delay_compensation_utils.cpp
  src/vehicle_models/vehicle_kinematic_error_model.cpp)


set(CDOB_LIB_INCLUDE_DIRS
  include)

ament_auto_add_library(${CDOB_LIB} SHARED
  ${CDOB_LIB_SOURCE}
  ${CDOB_LIB_HEADERS})

target_compile_options(${CDOB_LIB} PUBLIC -Wno-error=old-style-cast)

target_link_libraries(${CDOB_LIB}
  ${EIGEN3_LIBRARIES})

install(DIRECTORY include
  DESTINATION include)

install(TARGETS ${CDOB_LIB}
  EXPORT ${CDOB_LIB}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION
  INCLUDES DESTINATION include)

## -----------------  DELAY COMPENSATION NODE ----------------------------------------

set(DELAY_COMPENSATION_NODE communication_delay_compensation_node)
ament_auto_add_library(${DELAY_COMPENSATION_NODE} SHARED
  include/communication_delay_compensator_node.hpp
  src/communication_delay_compensator_node.cpp)

target_link_libraries(${DELAY_COMPENSATION_NODE}
  ${CDOB})

##------------ TYPE SUPPORT ----------------------------------------------------

set(MSG_FILES
  "msg/ControllerErrorReport.idl"
  "msg/DelayCompensationDebug.idl"
  "msg/DelayCompensationRefs.idl")

rosidl_generate_interfaces(${PROJECT_NAME}
  ${MSG_FILES}
  ADD_LINTER_TESTS
  DEPENDENCIES builtin_interfaces)

if(${rosidl_cmake_VERSION} VERSION_LESS 2.5.0)
  rosidl_target_interfaces(${DELAY_COMPENSATION_NODE}
    ${PROJECT_NAME} "rosidl_typesupport_cpp")

  rosidl_target_interfaces(${CDOB_LIB}
    ${PROJECT_NAME} "rosidl_typesupport_cpp")

else()
  rosidl_get_typesupport_target(
    cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
  target_link_libraries(${DELAY_COMPENSATION_NODE} "${cpp_typesupport_target}")
  target_link_libraries(${CDOB_LIB} "${cpp_typesupport_target}")

endif()

##------------- REGiSTER NODE ---------------------------------------------------

rclcpp_components_register_node(${DELAY_COMPENSATION_NODE}
  PLUGIN "observers::CommunicationDelayCompensatorNode"
  EXECUTABLE ${DELAY_COMPENSATION_NODE}_exe)

## ----------------- Library Tests----------------------------------------------

if(BUILD_TESTING)
  #  include(FetchContent)
  #  FetchContent_Declare(
  #    googletest
  #    # Specify the commit you depend on and update it regularly.
  #    URL https://github.com/google/googletest/archive/5376968f6948923e2411081fd9372e71a59d8e77.zip
  #  )
  #  # For Windows: Prevent overriding the parent project's compiler/linker settings
  #  set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
  #  FetchContent_MakeAvailable(googletest)

  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gtest REQUIRED)
  set(CDC_TESTs communication_delay_compensator_tests)

  ament_add_gtest(${CDC_TESTs}
    test/include/cdc_test_node.hpp
    test/include/cdc_test_utils.hpp
    test/src/cdc_test_node.cpp
    test/cdc_node_tests.cpp)

  ament_target_dependencies(${CDC_TESTs} fake_test_node)
  autoware_set_compile_options(${CDC_TESTs})

  target_include_directories(${CDC_TESTs} PRIVATE include test/include)
  target_link_libraries(${CDC_TESTs}
    ${CDOB_LIB}
    ${DELAY_COMPENSATION_NODE}
    ${EIGEN3_LIBRARIES})

endif()

# -------------  PROTOTYPES ____________________________
ament_auto_package(INSTALL_TO_SHARE
  launch
  param)