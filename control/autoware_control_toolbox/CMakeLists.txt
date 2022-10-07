cmake_minimum_required(VERSION 3.10)
project(autoware_control_toolbox)

find_package(autoware_cmake REQUIRED)
autoware_package()
find_package(fmt)

##----------------- Library headers and sources -------------------------------

##------------- CONTROL LIB  ----------------------------------

set(CONTROL_LIB autoware_control_toolbox_lib)

ament_auto_add_library(${CONTROL_LIB} SHARED
  DIRECTORY src/
)

target_link_libraries(${CONTROL_LIB}
  ${EIGEN3_LIBRARIES}
  ${Boost_LIBRARIES}
  fmt::fmt stdc++fs)

##----------------- Library Tests----------------------------------------------

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gtest REQUIRED)

  set(ACT_TESTs autoware_control_toolbox_tests)

  ament_add_gtest(${ACT_TESTs}
    test/act_test_suite.hpp
    test/tests_control_modules.cpp
    test/tests_spline_modules.cpp
    test/tests_util_modules.cpp)

  target_link_libraries(${ACT_TESTs}
    ${CONTROL_LIB}
    ${EIGEN3_LIBRARIES})
endif()

ament_auto_package(INSTALL_TO_SHARE)

#-------------------------  PROTOTYPES __________________________________________
## CONTROL
ament_auto_add_executable(main_tf usage_examples/control/main_tf.cpp)
ament_auto_add_executable(main_pade usage_examples/control/main_pade.cpp)
ament_auto_add_executable(main_tf2ss usage_examples/control/main_tf2ss.cpp)
ament_auto_add_executable(main_tf_algebra usage_examples/control/main_tf_algebra.cpp)
ament_auto_add_executable(main_balance usage_examples/control/main_balance.cpp)
ament_auto_add_executable(main_signal_generator usage_examples/control/main_signal_generator.cpp)

##---------------------- SPLINEs ------------------------------------------------

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

add_executable(main_1d_interp usage_examples/helper_functions/main_linear_1dinterp.cpp)
target_link_libraries(main_1d_interp ${CONTROL_LIB} fmt::fmt stdc++fs)

add_executable(main_diff_matt usage_examples/helper_functions/main_difference_matrix.cpp)
target_link_libraries(main_diff_matt ${CONTROL_LIB} fmt::fmt stdc++fs)
