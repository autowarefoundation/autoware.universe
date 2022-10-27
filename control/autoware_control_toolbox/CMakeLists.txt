cmake_minimum_required(VERSION 3.10)
project(autoware_control_toolbox)

# FMT
find_package(fmt)

# require that dependencies from package.xml be available
find_package(autoware_cmake REQUIRED)
autoware_package()


##------------ Library headers and sources --------------------

##------------- CONTROL LIB  ----------------------------------

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
  src/utils_act/writetopath.cpp)

set(CONTROL_LIB_HEADERS
  #CONTROL
  include/control/act_definitions.hpp
  include/autoware_control_toolbox.hpp
  include/control/balance.hpp
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
  include/utils_act/timekeep.hpp)

set(CONTROL_LIB_INCLUDE_DIRS
  include
  include/control
  include/splines
  include/utils_act)

##------------- Whole LIBRARY COMPILE --------------------

ament_auto_add_library(${CONTROL_LIB} SHARED
  ${CONTROL_LIB_SOURCE}
  ${CONTROL_LIB_HEADERS})
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
  INCLUDES DESTINATION include)

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

  # ament_target_dependencies(${ACT_TESTs} ${CONTROL_LIB})
  # autoware_set_compile_options(${ACT_TESTs})

  target_link_libraries(${ACT_TESTs}
    ${CONTROL_LIB}
    ${EIGEN3_LIBRARIES})
endif()

#ament package generation and installing
ament_auto_package(INSTALL_TO_SHARE)

