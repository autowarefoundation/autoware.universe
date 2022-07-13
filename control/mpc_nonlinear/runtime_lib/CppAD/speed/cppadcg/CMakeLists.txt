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
# Build the speed/cppadcg directory tests
# Inherit build type from ../CMakeList.txt
#
assert ( include_cppadcg )
#
# Adds flags to the compiler command line for sources in the current directory
# and below. This command can be used to add any flags, but it was originally
# intended to add preprocessor definitions.
ADD_DEFINITIONS("-DCPPAD_CPPADCG_SPEED")

# Local include directories to search (not in package_prefix/includdir)
INCLUDE_DIRECTORIES( ${CMAKE_CURRENT_SOURCE_DIR}/../src )

SET(source_list ../main.cpp
    det_lu.cpp
    det_minor.cpp
    mat_mul.cpp
    ode.cpp
    poly.cpp
    sparse_hessian.cpp
    sparse_jacobian.cpp
)
set_compile_flags( speed_cppadcg "${cppad_debug_which}" "${source_list}" )
#
ADD_EXECUTABLE(speed_cppadcg EXCLUDE_FROM_ALL ${source_list} )

# cppadcg is an include file only library
TARGET_LINK_LIBRARIES(speed_cppadcg
    speed_src
    ${cppad_lib}
    ${colpack_libs}
)


# Add the check_speed_cppadcg target
SET(random_seed 54321)
ADD_CUSTOM_TARGET(check_speed_cppadcg
    speed_cppadcg correct ${random_seed} onetape
    DEPENDS speed_cppadcg speed_src
)
MESSAGE(STATUS "make check_speed_cppadcg: available")

# Change check depends in parent environment
add_to_list(check_speed_depends check_speed_cppadcg)
SET(check_speed_depends "${check_speed_depends}" PARENT_SCOPE)
