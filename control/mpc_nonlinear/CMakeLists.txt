cmake_minimum_required(VERSION 3.14)
project(mpc_nonlinear)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_STANDARD 99)

# require that dependencies from package.xml be available
find_package(ament_cmake_auto REQUIRED)
find_package(rosidl_default_generators REQUIRED)

ament_auto_find_build_dependencies(REQUIRED
  ${${PROJECT_NAME}_BUILD_DEPENDS}
  ${${PROJECT_NAME}_BUILDTOOL_DEPENDS})

find_package(Eigen3 CONFIG)

# Release, Debug, RelWithDebInfo, MinSizeRel
#set(CMAKE_BUILD_TYPE "Release")
#set(CMAKE_CXX_FLAGS_RELEASE "-O3")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -std=c++17 -Wall -Wfatal-errors")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -pthread -std=c++17 -Wall -Wfatal-errors")

# -------------  FLAGS ---------------------------------------------------
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(
    -Wall
    -Wextra
    -Wpedantic
    # -Wuseless-cast
    # -Wno-unused-parameter
    # -Wno-unused-variable
    # -Wno-narrowing
    # -Wno-error=unused-but-set-variable
    # -Wno-conversion
    # -Wno-sign-conversion
    # -Wno-sign-compare
    -Wdouble-promotion
    # -Wfloat-equal
    -Waddress
    -fvisibility=default
    -fPIC)
endif()

# Clang tidy
if(TIDY_WITH_CLANG)
  string(
    CONCAT CMAKE_CXX_CLANG_TIDY
    "clang-tidy;"
    "-checks=-*,"
    "bugprone-*,"
    "cert-*,"
    "cppcoreguidelines-*,"
    "clang-analyze-*,"
    "google-*,"
    "hicpp-*,"
    "modernize-*,"
    "performance-*,"
    "readability-*")
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
  set(CMAKE_POSITION_INDEPENDENT_CODE ON)
endif()

## ------------------ External Library Dependencies ------------------------
# For Eigen vectorization.
include(CheckCXXCompilerFlag)

check_cxx_compiler_flag("-march=native" COMPILER_SUPPORTS_MARCH_NATIVE)
if(COMPILER_SUPPORTS_MARCH_NATIVE)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native") #  -mtune=native -mavx2 -mfma
  message(STATUS "Enabling MARCH NATIVE ")
endif()


### CppAD
#message("\n### Configuring CppAD ###")
#file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/external_libs/CppAD)
#
#execute_process(COMMAND cmake ${CMAKE_CURRENT_LIST_DIR}/external_libs/CppAD
#  WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/external_libs/CppAD)
#
## CppADCodeGen
#message("\n### Configuring CppADCodeGen ###")
#file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/external_libs/CppADCodeGen)
#
#execute_process(COMMAND cmake -DCPPAD_HOME=${CMAKE_CURRENT_LIST_DIR}/external_libs/CppAD/include
#  ${CMAKE_CURRENT_LIST_DIR}/external_libs/CppADCodeGen
#  WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/external_libs/CppADCodeGen)

find_package(CppAD QUIET)
if(CPPAD_FOUND)
  set(CPPAD ON)
  list(APPEND nmpc_node_COMPILE_DEFINITIONS CPPAD)
  list(APPEND nmpc_node_target_include_dirs ${CPPAD_INCLUDE_DIRS})
  msg(STATUS "CppAD is found ...")
else()
  message(STATUS "Could not find CppAD")
endif()

find_package(CppADCG QUIET)
if(CPPADCG_FOUND)
  set(CPPADCG ON)
  list(APPEND nmpc_core_COMPILE_DEFINITIONS CPPADCG)
  list(APPEND nmpc_core_target_include_dirs ${CPPADCG_INCLUDE_DIRS})
  msg(STATUS "CppADCodeGen is found ...")
else()
  message(STATUS "Could not find CppADCodeGen ")
endif()


# ------------ LIBRARIES --------------------------------
message(STATUS "osqp-cpp: Adding osqp-cpp library...")
ament_auto_add_library(osqp-cpp src/osqp_google/osqp++.cpp)

## ----------------- Nonlinear MPC Library headers and sources ---------------
# ---------------------- Vehicle Models --------------------------------------

set(VEHICLE_MODEL_SOURCE
  src/vehicle_models/vehicle_dynamics_base.cpp
  src/vehicle_models/vehicle_models_base.cpp
  src/vehicle_models/kinematic_vehicle_model.cpp)

set(VEHICLE_MODEL_INCLUDE_DIR include/vehicle_models)

# ------------------ NMPC core library --------------------------------------

set(NMPC_CORE mpc_nonlinear_lib_core)

set(NONLINEAR_MPC_CORE_SOURCE
  src/nonlinear_mpc_core/nmpc_data_discretization.cpp
  src/nonlinear_mpc_core/nmpc_data_trajectory.cpp
  src/nonlinear_mpc_core/data_and_parameter_container.cpp
  src/nonlinear_mpc_core/nmpc_core.cpp
  src/nonlinear_mpc_core/nmpc_discretization.cpp
  src/nonlinear_mpc_core/nmpc_optimization.cpp
  src/nonlinear_mpc_core/nmpc_simulation.cpp
  src/nonlinear_mpc_core/initialization_lpv.cpp
  src/utils/nmpc_utils.cpp)

set(NONLINEAR_MPC_CORE_INCLUDE_DIR
  include/nonlinear_mpc_core
  include/utils
  include/vehicle_models)


ament_auto_add_library(${NMPC_CORE}
  ${NONLINEAR_MPC_CORE_SOURCE}
  ${VEHICLE_MODEL_SOURCE}
  include/external)

target_link_libraries(${NMPC_CORE}
  osqp-cpp
  pthread)

target_compile_definitions(${NMPC_CORE} PUBLIC ${nmpc_core_COMPILE_DEFINITIONS})
target_include_directories(${NMPC_CORE} PUBLIC ${nmpc_node_target_include_dirs})

# -------------- NMPC Node ---------------------------------------------
set(NMPC_NODE mpc_nonlinear_node)
ament_auto_add_library(${NMPC_NODE} SHARED
  src/nonlinear_mpc_node/nonlinear_mpc_node.cpp
  src/nonlinear_mpc_node/nonlinear_mpc_node_visualization.cpp
  src/nonlinear_mpc_node/nonlinear_mpc_state_machine.cpp)

#--------------- NMPC Core ------------------------------------------------------
set(NONLINEAR_MPC_CORE_SOURCE
  src/nonlinear_mpc_core/nmpc_core.cpp)

set(NONLINEAR_MPC_CORE_INCLUDE_DIR
  include/nonlinear_mpc_core
  include/utils
  include/vehicle_models)


#--------------- NMPC Messages -----------------------------------------------------
set(MSG_FILES
  "msg/NonlinearMPCPerformanceReport.idl")

rosidl_generate_interfaces(${PROJECT_NAME}
  ${MSG_FILES}
  ADD_LINTER_TESTS
  DEPENDENCIES builtin_interfaces)

ament_export_dependencies(rosidl_default_runtime)


if(${rosidl_cmake_VERSION} VERSION_LESS 2.5.0)

  rosidl_target_interfaces(${NMPC_NODE}
    ${PROJECT_NAME}
    "rosidl_typesupport_cpp")

  target_link_libraries(${NMPC_NODE} "${cpp_typesupport_target}")
  target_link_libraries(${NMPC_CORE} "${cpp_typesupport_target}")

else()
  rosidl_get_typesupport_target(
    cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

  target_link_libraries(${NMPC_NODE} "${cpp_typesupport_target}")
  target_link_libraries(${NMPC_CORE} "${cpp_typesupport_target}")

endif()

# ------------- REGISTER and INSTALL ------------------------------------------

rclcpp_components_register_node(${NMPC_NODE}
  PLUGIN "ns_mpc_nonlinear::NonlinearMPCNode"
  EXECUTABLE ${NMPC_NODE}_exe)

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
  set(NMPC_TESTs nonlinear_mpc_tests)

  ament_add_gtest(${NMPC_TESTs}
    test/include/nonlinear_mpc_test_node.hpp
    test/include/nmpc_test_utils.hpp
    test/src/nonlinear_mpc_test_node.cpp
    test/src/test_nmpc_node.cpp)
  #test/src/test_google_osqp.cpp)

  ament_target_dependencies(${NMPC_TESTs} fake_test_node)
  autoware_set_compile_options(${NMPC_TESTs})

  target_include_directories(${NMPC_TESTs} PRIVATE include test/include)
  target_link_libraries(${NMPC_TESTs}
    ${NMPC_CORE}
    ${NMPC_NODE}
    osqp-cpp
    ${EIGEN3_LIBRARIES})

endif()

# Install to share.
ament_auto_package(INSTALL_TO_SHARE
  launch
  config
  msg
  params)

