cmake_minimum_required(VERSION 3.10)
project(autoware_control_toolbox)

# Example https://github.com/tier4/autoware.universe/blob/b6beeece906024794c708c1d4dc6051cd5e270fc/control/trajectory_follower/CMakeLists.txt#L105
# require that dependencies from package.xml be available

# Release, Debug, RelWithDebInfo, MinSizeRel
set(CMAKE_BUILD_TYPE "Debug")

if (NOT CMAKE_BUILD_TYPE MATCHES Debug)
    add_definitions(-DNDEBUG)
endif ()

if (NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
    set(CMAKE_CXX_EXTENSIONS OFF)
    set(CMAKE_POSITION_INDEPENDENT_CODE ON)
endif ()

if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif ()

#set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

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


if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(
            -Wall
            -Wextra
            -Wpedantic
            -Wno-unused-parameter
            # -Wconversion
            -Wdouble-promotion
            # -Wfloat-equal
            -Waddress
            -fvisibility=default) # visibility hidden or default is set here.
endif ()

## Dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

##  ----------------- Required Packages ---------------------------------------
find_package(Eigen3 REQUIRED)

# FMT
find_package(fmt)

# BOOST
find_package(Boost REQUIRED)

# EIGEN
#find_package(Eigen3 3.3.7 REQUIRED NO_MODULE)

## ----------------- Library headers and sources -------------------------------
set(CONTROL_LIB autoware_control_toolbox_lib)

set(CONTROL_LIB_SOURCE
        src/utils_act/act_utils.cpp
        src/utils_act/act_utils_eigen.cpp
        src/autoware_control_toolbox.cpp
        src/tf_algebra.cpp
        src/utils_act/state_space.cpp
        src/utils_act/transfer_functions.cpp
        src/utils_act/balance.cpp
        )

set(CONTROL_LIB_HEADERS
        include/utils_act/act_definitions.hpp
        include/autoware_control_toolbox.hpp
        include/utils_act/act_utils.hpp
        include/utils_act/balance.hpp
        include/utils_act/act_utils_eigen.hpp
        include/visibility_control.hpp
        include/tf_algebra.hpp
        include/utils_act/state_space.hpp
        include/utils_act/transfer_functions.hpp
        )

set(CONTROL_LIB_INCLUDE_DIRS
        include
        include/utils)

ament_auto_add_library(${CONTROL_LIB} SHARED
        ${CONTROL_LIB_SOURCE}
        ${CONTROL_LIB_HEADERS}
        )


#ament_export_targets(${CONTROL_LIB} HAS_LIBRARY_TARGET)

#target_include_directories(${CONTROL_LIB}
#        PUBLIC
#        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/utils>
#        $<INSTALL_INTERFACE:include>
#        $<INSTALL_INTERFACE:include/utils>)

#ament_export_targets(${CONTROL_LIB} HAS_LIBRARY_TARGET)
#ament_export_libraries(${CONTROL_LIB})
#ament_export_include_directories(include include/utils)


target_link_libraries(${CONTROL_LIB}
        ${EIGEN3_LIBRARIES}
        ${Boost_LIBRARIES})

install(DIRECTORY include
        DESTINATION include)

install(TARGETS ${CONTROL_LIB}
        EXPORT ${CONTROL_LIB}
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION
        INCLUDES DESTINATION include
        )

#ament_export_include_directories(include include/utils)
#ament_export_libraries(${CONTROL_LIB})

## ----------------- Library Tests----------------------------------------------

if (BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
    find_package(ament_cmake_gtest REQUIRED)

    #    ament_add_gtest(
    #            mpc_nonlinear_tests
    #            src/nonlinear_mpc_node/nonlinear_mpc_node.cpp
    #            test/src/test_nmpc_node.cpp)
    #
    #    target_include_directories(mpc_nonlinear_tests PRIVATE include test/include)
    #    target_link_libraries(mpc_nonlinear_tests mpc_nonlinear_lib_core bspline_lib
    #            osqp-cpp ${EIGEN3_LIBRARIES})
endif ()


#ament package generation and installing
# ament_auto_package(INSTALL_TO_SHARE)
# ament_auto_package(INSTALL_TO_SHARE config launch)


target_compile_options(${CONTROL_LIB} PUBLIC "-O0" PRIVATE -Wall)
ament_auto_package(CONFIG_EXTRAS_POST "${CONTROL_LIB}-extras.cmake")
# --------------- Other Installation Macros ------------


# -------------  PROTOTYPES ____________________________

## TF example
ament_auto_add_executable(main_tf usage_examples/main_tf.cpp)
ament_auto_add_executable(main_pade usage_examples/main_pade.cpp)
ament_auto_add_executable(main_tf2ss usage_examples/main_tf2ss.cpp)
ament_auto_add_executable(main_tf_algebra usage_examples/main_tf_algebra.cpp)
#target_link_libraries(main_tf fmt::fmt stdc++fs ${AUTOWARE_CONTROL_TOOLBOX_LIB})
#target_link_directories(main_tf PUBLIC ${AUTOWARE_CONTROL_LIB_INCLUDE_DIR})
#target_include_directories(main_tf PRIVATE include include/utils)
