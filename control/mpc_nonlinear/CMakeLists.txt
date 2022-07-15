cmake_minimum_required(VERSION 3.14)
project(mpc_nonlinear)

# require that dependencies from package.xml be available
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies(REQUIRED
        ${${PROJECT_NAME}_BUILD_DEPENDS}
        ${${PROJECT_NAME}_BUILDTOOL_DEPENDS})

find_package(Eigen3 CONFIG)

# set(CMAKE_CXX_STANDARD 17)
# Release, Debug, RelWithDebInfo, MinSizeRel
#set(CMAKE_BUILD_TYPE "Release")
#set(CMAKE_CXX_FLAGS_RELEASE "-O3")

# -------------  FLAGS ---------------------------------------------------
if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(
            -Wall
            -Wextra
            -Wpedantic
            -Wno-unused-parameter
            -Wno-unused-variable
            -Wno-narrowing
            -Wno-error=unused-but-set-variable
            # -Wno-conversion
            # -Wno-sign-conversion
            -Wno-sign-compare
            #  -Wdouble-promotion
            # -Wfloat-equal
            -Waddress
            -fvisibility=default
            -fPIC) # visibility hidden or default is set here.
endif ()

# Clang tidy
if (TIDY_WITH_CLANG)
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
endif ()


if (NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
    set(CMAKE_CXX_EXTENSIONS OFF)
    set(CMAKE_POSITION_INDEPENDENT_CODE ON)
endif ()


## ------------------ External Library Dependencies ------------------------
# For Eigen vectorization.
include(CheckCXXCompilerFlag)

check_cxx_compiler_flag("-march=native" COMPILER_SUPPORTS_MARCH_NATIVE)
if (COMPILER_SUPPORTS_MARCH_NATIVE)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native")
endif ()


# CppAD
#message("\n### Configuring CppAD ###")
#file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/runtime_lib/CppAD)
#execute_process(COMMAND cmake ${CMAKE_CURRENT_LIST_DIR}/runtime_lib/CppAD
#	WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/runtime_lib/CppAD)

## CppADCodeGen
#message("\n### Configuring CppADCodeGen ###")
#file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/runtime_lib/CppADCodeGen)
#execute_process(COMMAND cmake -DCPPAD_HOME=${CMAKE_CURRENT_LIST_DIR}/lib/CppAD/include
#	${CMAKE_CURRENT_LIST_DIR}/runtime_lib/CppADCodeGen
#	WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/runtime_lib/CppADCodeGen)

# --------------- TEMPORARY ADD-ONs ---------------------
## BOOST
#find_package(Boost REQUIRED)
#
## EIGEN
#find_package(Eigen3 3.3.7 REQUIRED NO_MODULE)
#find_package(Eigen3 CONFIG)

## OSQP
#find_package(osqp REQUIRED) # make install google-osqp and copy headers and source into current project
#
#if(${Eigen3_FOUND})
#	message("Eigen FOUND ...")
#endif(${Eigen3_FOUND})
#
#if(${osqp_FOUND})
#	message("OSQP FOUND ...")
#	include_directories(osqp_INCLUDE_DIRS)
#endif(${osqp_FOUND})
#
#if(${Boost_FOUND})
#	#    include_directories(Boost_INCLUDE_DIRS)
#	message("Boost Found")
#endif()

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
        src/nonlinear_mpc_core/nmpc_kalman_filter.cpp
        src/nonlinear_mpc_core/initialization_lpv.cpp
        src/utils/nmpc_utils.cpp
        )

set(NONLINEAR_MPC_CORE_INCLUDE_DIR
        include/nonlinear_mpc_core
        include/utils
        include/vehicle_models
        )


ament_auto_add_library(${NMPC_CORE}
        ${NONLINEAR_MPC_CORE_SOURCE}
        ${VEHICLE_MODEL_SOURCE}
        include/external)

target_link_libraries(mpc_nonlinear_lib_core
        osqp-cpp
        #	Eigen3::Eigen
        )

# -------------- NMPC Node ---------------------------------------------
set(NMPC_NODE mpc_nonlinear_node)
ament_auto_add_library(mpc_nonlinear_node SHARED
        src/nonlinear_mpc_node/nonlinear_mpc_node.cpp
        src/nonlinear_mpc_node/nonlinear_mpc_node_visualization.cpp
        src/nonlinear_mpc_node/nonlinear_mpc_state_machine.cpp
        )

#--------------- NMPC Core ------------------------------------------------------
set(NONLINEAR_MPC_CORE_SOURCE
        src/nonlinear_mpc_core/nmpc_core.cpp)

set(NONLINEAR_MPC_CORE_INCLUDE_DIR
        include/nonlinear_mpc_core
        include/utils
        include/vehicle_models)


# ------------- REGISTER and INSTALL ------------------------------------------

rclcpp_components_register_node(${NMPC_NODE}
        PLUGIN "ns_mpc_nonlinear::NonlinearMPCNode"
        EXECUTABLE ${NMPC_NODE}_exe
        )

ament_auto_package(INSTALL_TO_SHARE
        launch
        config)


# -------------  PROTOTYPES ---------------------------------------------------
#add_executable(main_vehicle main_vehicle_test.cpp)
#target_link_libraries(main_vehicle mpc_nonlinear_lib_core)
