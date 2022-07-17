cmake_minimum_required(VERSION 3.10)
project(communication_delay_compensator)

# require that dependencies from package.xml be available
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies(REQUIRED
        ${${PROJECT_NAME}_BUILD_DEPENDS}
        ${${PROJECT_NAME}_BUILDTOOL_DEPENDS})


if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
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
endif ()

## Release, Debug, RelWithDebInfo, MinSizeRel
#set(CMAKE_BUILD_TYPE "Release")
#
#if (CMAKE_BUILD_TYPE MATCHES RELEASE)
#    set(CMAKE_CXX_FLAGS_RELEASE "-O3")
#    message("In release mode ...")
#endif ()

## ----------------- Library headers and sources -------------------------------
set(CDOB_LIB_INCLUDE_DIRS
        include)

set(CDOB_LIB delay_compensation_lib)

set(CDOB_LIB_HEADERS
        include/communication_delay_compensator_core.hpp
        include/utils_delay_observer/delay_compensation_utils.hpp
        include/utils_delay_observer/delay_compensation_utils.hpp
        include/vehicle_models/vehicle_definitions.hpp
        include/vehicle_models/vehicle_kinematic_error_model.hpp
        include/adaptive_parameter_estimators/adaptive_parameter_estimation.hpp

        #Integration
        include/external/eigen_integration_helper.hpp
        include/integration/integrate_eigen_states.hpp
        )


set(CDOB_LIB_SOURCE
        src/communication_delay_compensator_core.cpp
        src/utils_delay_observer/delay_compensation_utils.cpp
        src/utils_delay_observer/delay_compensation_utils.cpp
        src/vehicle_models/vehicle_kinematic_error_model.cpp
        src/adaptive_parameter_estimators/adaptive_parameter_estimation.cpp

        # Integration
        src/integration/integrate_eigen_states.cpp
        )


set(CDOB_LIB_INCLUDE_DIRS
        include)

ament_auto_add_library(${CDOB_LIB} SHARED
        ${CDOB_LIB_SOURCE}
        ${CDOB_LIB_HEADERS}
        )

target_compile_options(${CDOB_LIB} PRIVATE -Wno-error=old-style-cast)

target_link_libraries(${CDOB_LIB}
        ${EIGEN3_LIBRARIES})

install(DIRECTORY include
        DESTINATION include)

install(TARGETS ${CDOB_LIB}
        EXPORT ${CDOB_LIB}
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION
        INCLUDES DESTINATION include
        )
## -----------------  DELAY COMPENSATION NODE ----------------------------------------
set(DELAY_COMPENSATION_NODE communication_delay_compensator)
ament_auto_add_library(${DELAY_COMPENSATION_NODE} SHARED
        include/communication_delay_compensator_node.hpp
        src/communication_delay_compensator_node.cpp
        )

target_link_libraries(${DELAY_COMPENSATION_NODE}
        ${CDOB})

rclcpp_components_register_node(${DELAY_COMPENSATION_NODE}
        PLUGIN "observers::CommunicationDelayCompensatorNode"
        EXECUTABLE ${DELAY_COMPENSATION_NODE}_exe
        )
## ----------------- Library Tests----------------------------------------------

if (BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
    find_package(ament_cmake_gtest REQUIRED)
endif ()

# -------------  PROTOTYPES ____________________________
ament_auto_package(INSTALL_TO_SHARE
        launch
        param)


##  Usage Examples
## Type erasure example
ament_auto_add_executable(main_type_erasure usage_examples/main_polymorphisim.cpp)
target_link_libraries(main_type_erasure ${CDOB})


## ODE integration
ament_auto_add_executable(main_ode usage_examples/main_ode.cpp)
target_link_libraries(main_ode ${CDOB})

 
