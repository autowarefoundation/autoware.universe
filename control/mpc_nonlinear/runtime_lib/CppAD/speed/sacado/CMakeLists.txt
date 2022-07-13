# -----------------------------------------------------------------------------
# CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-20 Bradley M. Bell
#
# CppAD is distributed under the terms of the
#              Eclipse Public License Version 2.0.
#
# This Source Code may also be made available under the following
# Secondary License when the conditions for such availability set forth
# in the Eclipse Public License, Version 2.0 are satisfied:
#       GNU General Public License, Version 2.0 or later.
# -----------------------------------------------------------------------------
# Build the speed/sacado directory tests
# Inherit build type from ../CMakeList.txt

# assert sacado_prefix is defined
assert( sacado_prefix )

# Adds flags to the compiler command line for sources in the current directory
# and below. This command can be used to add any flags, but it was originally
# intended to add preprocessor definitions.
ADD_DEFINITIONS("-DCPPAD_SACADO_SPEED -DRAD_AUTO_AD_Const")

# Local include directories to search (not in package_prefix/includdir)
INCLUDE_DIRECTORIES( ${CMAKE_CURRENT_SOURCE_DIR}/../src )

# add_executable(<name> [WIN32] [MACOSX_BUNDLE] [EXCLUDE_FROM_ALL]
#                 source1 source2 ... sourceN
# )
SET(source_list ../main.cpp
    det_lu.cpp
    det_minor.cpp
    mat_mul.cpp
    ode.cpp
    poly.cpp
    sparse_hessian.cpp
    sparse_jacobian.cpp
)
set_compile_flags( speed_sacado "${cppad_debug_which}" "${source_list}" )
#
ADD_EXECUTABLE( speed_sacado EXCLUDE_FROM_ALL ${source_list} )

# determine libraries necessary to link Sacado
SET(config_file "${sacado_prefix}/include/Makefile.export.Sacado")
IF(NOT EXISTS ${config_file})
    MESSAGE(FATAL_ERROR
"the file sacado_prefix/include/Makefile.export.Sacado does not exist
where sacado_prefix=${sacado_prefix}"
    )
ENDIF(NOT EXISTS ${config_file})
SET(cmd "grep" "Sacado_LIBRARIES=" "${config_file}")
EXECUTE_PROCESS(
    COMMAND         ${cmd}
    OUTPUT_VARIABLE temp1
)
STRING(REGEX REPLACE "Sacado_LIBRARIES= *-l" "" temp2  ${temp1})
STRING(STRIP ${temp2} temp3)
STRING(REGEX REPLACE " *-l" ";" sacado_lib_list ${temp3})
#
# include the dl library if it is in the system path
CHECK_LIBRARY_EXISTS( dl dlopen "${CMAKE_SYSTEM_LIBRARY_PATH}" dl_found )
IF( dl_found )
    TARGET_LINK_LIBRARIES(speed_sacado speed_src ${sacado_lib_list} dl)
ELSE( dl_found )
    TARGET_LINK_LIBRARIES(speed_sacado speed_src ${sacado_lib_list})
ENDIF( dl_found )
#
# Add the check_speed_sacado target
ADD_CUSTOM_TARGET(check_speed_sacado
    speed_sacado correct 54321
    DEPENDS speed_sacado speed_src
)
MESSAGE(STATUS "make check_speed_sacado: available")

# Change check depends in parent environment
add_to_list(check_speed_depends check_speed_sacado)
SET(check_speed_depends "${check_speed_depends}" PARENT_SCOPE)
