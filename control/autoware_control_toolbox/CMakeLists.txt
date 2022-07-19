cmake_minimum_required(VERSION 3.10)
project(autoware_control_toolbox)

# FMT
find_package(fmt)

# require that dependencies from package.xml be available
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies(REQUIRED
        ${${PROJECT_NAME}_BUILD_DEPENDS}
        ${${PROJECT_NAME}_BUILDTOOL_DEPENDS}
        )
set(CMAKE_CXX_STANDARD 17)

# Release, Debug, RelWithDebInfo, MinSizeRel
#set(CMAKE_BUILD_TYPE "Debug")
#if (CMAKE_BUILD_TYPE MATCHES RELEASE)
#    set(CMAKE_CXX_FLAGS_RELEASE "-O3")
#    message("In release mode ...")
#endif ()


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
## ----------------- Library headers and sources -------------------------------

## ------------- CONTROL LIB  ----------------------------------

set(CONTROL_LIB autoware_control_toolbox_lib)

set(CONTROL_LIB_SOURCE
        #CONTROL
        src/utils_act/act_utils.cpp
        src/utils_act/act_utils_eigen.cpp
        src/autoware_control_toolbox.cpp
        src/control/tf_algebra.cpp
        src/control/state_space.cpp
        src/control/transfer_functions.cpp
        src/control/balance.cpp
        src/control/act_signal_builder.cpp

        #SPLINES
        src/splines/interpolating_spline_pcg.cpp
        src/splines/bsplines_smoother.cpp
        src/splines/bsplines_interpolator.cpp
        src/splines/bspline_interpolator_templated.cpp

        #UTILS
        src/utils_act/act_utils.cpp
        src/utils_act/act_utils_eigen.cpp
        src/utils_act/timekeep.cpp
        src/utils_act/writetopath.cpp
        )

set(CONTROL_LIB_HEADERS
        #CONTROL
        include/control/act_definitions.hpp
        include/autoware_control_toolbox.hpp
        include/control/balance.hpp
        include/visibility_control.hpp
        include/control/tf_algebra.hpp
        include/control/state_space.hpp
        include/control/transfer_functions.hpp
        include/control/act_signal_builder.hpp
        #SPLINES
        include/splines/bsplines_interpolator.hpp
        include/splines/interpolating_spline_pcg.hpp
        include/splines/bsplines_smoother.hpp
        include/splines/bspline_interpolator_templated.hpp
        #UTILS
        include/utils_act/act_utils_eigen.hpp
        include/utils_act/act_utils.hpp
        include/utils_act/writetopath.hpp
        include/utils_act/timekeep.hpp
        )

set(CONTROL_LIB_INCLUDE_DIRS
        include
        include/control
        include/splines
        include/utils_act)


## ------------- Whole LIBRARY COMPILE --------------------

ament_auto_add_library(${CONTROL_LIB} SHARED
        ${CONTROL_LIB_SOURCE}
        ${CONTROL_LIB_HEADERS}
        )
target_compile_options(${CONTROL_LIB} PRIVATE -Wno-error=old-style-cast)

target_link_libraries(${CONTROL_LIB}
        ${EIGEN3_LIBRARIES}
        ${Boost_LIBRARIES}
        fmt::fmt stdc++fs)

install(DIRECTORY include
        DESTINATION include)

install(TARGETS ${CONTROL_LIB}
        EXPORT ${CONTROL_LIB}
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION
        INCLUDES DESTINATION include
        )

## ----------------- Library Tests----------------------------------------------

if (BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
    find_package(ament_cmake_gtest REQUIRED)

endif ()

#ament package generation and installing
ament_auto_package(INSTALL_TO_SHARE)


# -------------------------  PROTOTYPES ____________________________

## CONTROL
ament_auto_add_executable(main_tf usage_examples/control/main_tf.cpp)
ament_auto_add_executable(main_pade usage_examples/control/main_pade.cpp)
ament_auto_add_executable(main_tf2ss usage_examples/control/main_tf2ss.cpp)
ament_auto_add_executable(main_tf_algebra usage_examples/control/main_tf_algebra.cpp)
ament_auto_add_executable(main_balance usage_examples/control/main_balance.cpp)
ament_auto_add_executable(main_signal_generator usage_examples/control/main_signal_generator.cpp)
#target_link_libraries(main_tf fmt::fmt stdc++fs ${AUTOWARE_CONTROL_TOOLBOX_LIB})
#target_link_directories(main_tf PUBLIC ${AUTOWARE_CONTROL_LIB_INCLUDE_DIR})
#target_include_directories(main_tf PRIVATE include include/utils)

## ---------------------- SPLINEs -----------------------------

add_executable(main_spline_pcg usage_examples/splines/main_spline_eigen_pcg.cpp)
target_link_libraries(main_spline_pcg ${CONTROL_LIB} fmt::fmt stdc++fs)

add_executable(main_bspline_smoother usage_examples/splines/main_bspline_smoother.cpp)
target_link_libraries(main_bspline_smoother ${CONTROL_LIB} fmt::fmt stdc++fs)

add_executable(main_bspline_interp usage_examples/splines/main_bspline_interpolator.cpp)
target_link_libraries(main_bspline_interp ${CONTROL_LIB} fmt::fmt stdc++fs)

add_executable(main_bspline_interp_template usage_examples/splines/main_bspline_interpolator_templated.cpp)
target_link_libraries(main_bspline_interp_template ${CONTROL_LIB} fmt::fmt stdc++fs)

add_executable(main_unwrap usage_examples/helper_functions/main_unwrap.cpp)
target_link_libraries(main_unwrap ${CONTROL_LIB} fmt::fmt stdc++fs)
