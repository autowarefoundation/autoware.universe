cmake_minimum_required(VERSION 3.14)
project(autoware_trajectory_modifier)

find_package(autoware_cmake REQUIRED)

autoware_package()

find_package(acados REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

find_package(ament_cmake_export_dependencies REQUIRED)
find_package(ament_cmake_export_link_flags REQUIRED)
ament_export_dependencies(acados)

get_target_property(_acados_location acados IMPORTED_LOCATION)
if(NOT _acados_location)
  get_target_property(_acados_location acados IMPORTED_LOCATION_RELEASE)
endif()
if(_acados_location)
  get_filename_component(_acados_lib_dir "${_acados_location}" DIRECTORY)
  # Quote each flag as one cmake argument; -L"/path" breaks the generated extras file.
  ament_export_link_flags("-L${_acados_lib_dir}" "-lacados")
else()
  # Fallback when acados is not exposed as an imported target (e.g. in-tree)
  set(_acados_lib_dir "$ENV{ACADOS_SOURCE_DIR}/lib")
  if(_acados_lib_dir AND EXISTS "${_acados_lib_dir}")
    ament_export_link_flags("-L${_acados_lib_dir}" "-lacados")
  endif()
endif()

# Temporal kinematic-bicycle acados solver: Python codegen at build time (add_custom_command).
# See src/acados_mpt/CMakeLists.txt — target acados_interface_temporal.
add_subdirectory(src/acados_mpt)
# Pose-only time-sequence acados solver (independent of Temporal MPT).
add_subdirectory(src/acados_time_sequence_raw)
pluginlib_export_plugin_description_file(${PROJECT_NAME} plugins.xml)

generate_parameter_library(trajectory_modifier_param
  param/trajectory_modifier_parameter_struct.yaml
)

ament_auto_add_library(autoware_trajectory_modifier_utils SHARED
  src/utils.cpp
  src/trajectory_modifier_utils/utils.cpp
  src/trajectory_modifier_utils/obstacle_stop_utils.cpp
  src/trajectory_modifier_utils/planning_factor_utils.cpp
)

ament_auto_add_library(autoware_trajectory_modifier_component SHARED
  src/trajectory_modifier.cpp
)

target_link_libraries(autoware_trajectory_modifier_component
  autoware_trajectory_modifier_utils
  trajectory_modifier_param
)

target_compile_options(autoware_trajectory_modifier_component PUBLIC
  -Wno-error=deprecated-declarations
)

autoware_agnocast_wrapper_register_node(autoware_trajectory_modifier_component
  PLUGIN "autoware::trajectory_modifier::TrajectoryModifier"
  EXECUTABLE autoware_trajectory_modifier_node
  ROS2_EXECUTOR SingleThreadedExecutor
  AGNOCAST_EXECUTOR CallbackIsolatedAgnocastExecutor
)

ament_auto_add_library(autoware_trajectory_optimizer_plugins SHARED
  src/trajectory_optimizer_plugins/trajectory_eb_smoother_optimizer.cpp
  src/trajectory_optimizer_plugins/trajectory_extender.cpp
  src/trajectory_optimizer_plugins/trajectory_kinematic_feasibility_enforcer.cpp
  src/trajectory_optimizer_plugins/trajectory_mpt_optimizer.cpp
  src/trajectory_optimizer_plugins/trajectory_temporal_mpt_optimizer.cpp
  src/trajectory_optimizer_plugins/trajectory_time_sequence_raw_optimizer.cpp
  src/trajectory_optimizer_plugins/trajectory_point_fixer.cpp
  src/trajectory_optimizer_plugins/trajectory_qp_smoother.cpp
  src/trajectory_optimizer_plugins/trajectory_spline_smoother.cpp
  src/trajectory_optimizer_plugins/trajectory_velocity_optimizer.cpp
  src/trajectory_optimizer_plugins/plugin_utils/continuous_jerk_smoother.cpp
  src/trajectory_optimizer_plugins/plugin_utils/trajectory_extender_utils.cpp
  src/trajectory_optimizer_plugins/plugin_utils/trajectory_mpt_optimizer_utils.cpp
  src/trajectory_optimizer_plugins/plugin_utils/trajectory_point_fixer_utils.cpp
  src/trajectory_optimizer_plugins/plugin_utils/trajectory_spline_smoother_utils.cpp
  src/trajectory_optimizer_plugins/plugin_utils/trajectory_velocity_optimizer_utils.cpp
  src/trajectory_optimizer_plugins/plugin_utils/time_sequence_trajectory_optimizer.cpp
  src/trajectory_optimizer_plugins/plugin_utils/road_border_avoidance.cpp
)
target_link_libraries(
  autoware_trajectory_optimizer_plugins
  acados_interface_temporal
  acados_interface_time_seq
  autoware_trajectory_modifier_utils
  tf2_geometry_msgs::tf2_geometry_msgs
  trajectory_modifier_param
)

# libqpOASES_e is normally only a transitive dependency of libacados. The dynamic
# linker then resolves it using libacados' RPATH (often empty), not this plugin's
# INSTALL_RPATH below — so dlopen fails without LD_LIBRARY_PATH. Link qpOASES
# directly so it appears as DT_NEEDED on this DSO; --no-as-needed keeps the link
# even though we reference no qpOASES symbols here.
if(TARGET qpOASES_e::qpOASES_e)
  set(_traj_opt_qpoases_lib qpOASES_e::qpOASES_e)
elseif(TARGET qpOASES_e)
  set(_traj_opt_qpoases_lib qpOASES_e)
else()
  find_library(_traj_opt_qpoases_lib_path NAMES qpOASES_e
    PATHS
      "${_acados_lib_dir}"
      "$ENV{ACADOS_SOURCE_DIR}/lib"
      /opt/acados/lib
    NO_DEFAULT_PATH)
  if(NOT _traj_opt_qpoases_lib_path)
    find_library(_traj_opt_qpoases_lib_path NAMES qpOASES_e)
  endif()
  if(_traj_opt_qpoases_lib_path)
    set(_traj_opt_qpoases_lib "${_traj_opt_qpoases_lib_path}")
  endif()
endif()
if(_traj_opt_qpoases_lib)
  if(UNIX AND NOT APPLE)
    target_link_options(autoware_trajectory_optimizer_plugins PRIVATE "LINKER:--no-as-needed")
  endif()
  target_link_libraries(autoware_trajectory_optimizer_plugins ${_traj_opt_qpoases_lib})
  if(UNIX AND NOT APPLE)
    target_link_options(autoware_trajectory_optimizer_plugins PRIVATE "LINKER:--as-needed")
  endif()
endif()
unset(_traj_opt_qpoases_lib_path CACHE)

# Ensure pluginlib/dlopen can resolve acados runtime deps (e.g. libqpOASES_e)
# in container environments where LD_LIBRARY_PATH is restricted.
if(_acados_lib_dir AND EXISTS "${_acados_lib_dir}")
  foreach(_target acados_interface_temporal acados_interface_time_seq autoware_trajectory_optimizer_plugins)
    set_property(TARGET ${_target} APPEND PROPERTY BUILD_RPATH "${_acados_lib_dir}")
    set_property(TARGET ${_target} APPEND PROPERTY INSTALL_RPATH "${_acados_lib_dir}")
  endforeach()
endif()

ament_auto_add_library(autoware_trajectory_modifier_plugins SHARED
  src/trajectory_modifier_plugins/external_velocity_limit.cpp
  src/trajectory_modifier_plugins/stop_point_fixer.cpp
  src/trajectory_modifier_plugins/obstacle_stop.cpp
  src/trajectory_modifier_plugins/velocity_modifier.cpp
  src/trajectory_modifier_plugins/surround_obstacle_stop.cpp
  src/trajectory_modifier_plugins/traffic_light_stop.cpp
  src/trajectory_modifier_plugins/map_velocity_limits.cpp
  src/trajectory_modifier_plugins/velocity_limits.cpp
  src/trajectory_modifier_plugins/model_planning_factor_id.cpp
)

target_link_libraries(autoware_trajectory_modifier_plugins
  autoware_trajectory_modifier_utils
  trajectory_modifier_param
)

option(BUILD_VELOCITY_LIMIT_SIMULATION_TESTS
  "Build and register the closed-loop velocity limit simulation tests" OFF)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  file(GLOB_RECURSE TEST_SOURCES tests/*.cpp)
  if(NOT BUILD_VELOCITY_LIMIT_SIMULATION_TESTS)
    list(REMOVE_ITEM TEST_SOURCES
      "${CMAKE_CURRENT_SOURCE_DIR}/tests/test_velocity_limit_simulation.cpp"
      "${CMAKE_CURRENT_SOURCE_DIR}/tests/test_velocity_limit_plugin_simulation.cpp"
    )
  endif()
  ament_add_ros_isolated_gtest(test_autoware_trajectory_modifier
    ${TEST_SOURCES}
  )
  target_link_libraries(test_autoware_trajectory_modifier
    autoware_trajectory_modifier_component
    autoware_trajectory_optimizer_plugins
    autoware_trajectory_modifier_plugins
    autoware_trajectory_modifier_utils
    acados_interface_time_seq
  )

  if(BUILD_VELOCITY_LIMIT_SIMULATION_TESTS)
    # Closed-loop simulations write CSV data here (overridden at run time by the
    # VELOCITY_LIMIT_SIMULATION_OUTPUT_DIR environment variable); the plot test renders it.
    set(VELOCITY_LIMIT_SIMULATION_OUTPUT_DIR "${CMAKE_CURRENT_BINARY_DIR}/velocity_limit_simulation")
    target_compile_definitions(test_autoware_trajectory_modifier PRIVATE
      VELOCITY_LIMIT_SIMULATION_OUTPUT_DIR="${VELOCITY_LIMIT_SIMULATION_OUTPUT_DIR}"
    )
    add_test(NAME plot_velocity_limit_simulation
      COMMAND python3 ${CMAKE_CURRENT_SOURCE_DIR}/tests/plot_velocity_limit_simulation.py
        ${VELOCITY_LIMIT_SIMULATION_OUTPUT_DIR}
    )
    set_tests_properties(plot_velocity_limit_simulation PROPERTIES
      DEPENDS test_autoware_trajectory_modifier
    )
  endif()
endif()

install(TARGETS acados_interface_temporal acados_interface_time_seq
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(FILES include/temporal_mpt/path_tracking_solver.hpp
  DESTINATION include/temporal_mpt)

set(TEMPORAL_MODEL "kinematic_bicycle_temporal")
set(TEMPORAL_GEN_DIR "${CMAKE_CURRENT_BINARY_DIR}/src/acados_mpt/c_generated_code")
install(FILES
  ${TEMPORAL_GEN_DIR}/acados_solver_${TEMPORAL_MODEL}.h
  ${TEMPORAL_GEN_DIR}/${TEMPORAL_MODEL}_model/${TEMPORAL_MODEL}_model.h
  DESTINATION include/autoware/trajectory_modifier/c_generated_code
  OPTIONAL
)

install(PROGRAMS
  scripts/temporal_mpt_debug_visualizer.py
  scripts/temporal_mpt_python_reference.py
  scripts/temporal_mpt_offline_retune.py
  scripts/temporal_mpt_offline_tuner.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_libraries(acados_interface_temporal acados_interface_time_seq)

ament_auto_package(
  INSTALL_TO_SHARE
  config
  launch
)
