cmake_minimum_required(VERSION 3.10)
project(autoware_control_toolbox)

find_package(autoware_cmake REQUIRED)
autoware_package()


## ----------------- Library headers and sources -------------------------------
set(CONTROL_LIB autoware_control_toolbox_lib)

set(CONTROL_LIB_SOURCE
        src/utils_act/act_utils.cpp
        src/utils_act/act_utils_eigen.cpp
        src/autoware_control_toolbox.cpp
        src/utils_act/tf_algebra.cpp
        src/utils_act/state_space.cpp
        src/utils_act/transfer_functions.cpp
        src/utils_act/balance.cpp
        src/utils_act/act_signal_builder.cpp
        )

set(CONTROL_LIB_HEADERS
        include/utils_act/act_definitions.hpp
        include/autoware_control_toolbox.hpp
        include/utils_act/act_utils.hpp
        include/utils_act/balance.hpp
        include/utils_act/act_utils_eigen.hpp
        include/visibility_control.hpp
        include/utils_act/tf_algebra.hpp
        include/utils_act/state_space.hpp
        include/utils_act/transfer_functions.hpp
        include/utils_act/act_signal_builder.hpp
        )

set(CONTROL_LIB_INCLUDE_DIRS
        include
        include/utils)


ament_auto_add_library(${CONTROL_LIB} SHARED
        ${CONTROL_LIB_SOURCE}
        ${CONTROL_LIB_HEADERS}
        )
target_compile_options(${CONTROL_LIB} PRIVATE -Wno-error=old-style-cast)

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

## ----------------- Library Tests----------------------------------------------

if (BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
    find_package(ament_cmake_gtest REQUIRED)

endif ()

#ament package generation and installing
ament_auto_package(INSTALL_TO_SHARE)


# -------------  PROTOTYPES ____________________________

## TF example
ament_auto_add_executable(main_tf usage_examples/main_tf.cpp)
ament_auto_add_executable(main_pade usage_examples/main_pade.cpp)
ament_auto_add_executable(main_tf2ss usage_examples/main_tf2ss.cpp)
ament_auto_add_executable(main_tf_algebra usage_examples/main_tf_algebra.cpp)
ament_auto_add_executable(main_balance usage_examples/main_balance.cpp)
ament_auto_add_executable(main_signal_generator usage_examples/main_signal_generator.cpp)
#target_link_libraries(main_tf fmt::fmt stdc++fs ${AUTOWARE_CONTROL_TOOLBOX_LIB})
#target_link_directories(main_tf PUBLIC ${AUTOWARE_CONTROL_LIB_INCLUDE_DIR})
#target_include_directories(main_tf PRIVATE include include/utils)
