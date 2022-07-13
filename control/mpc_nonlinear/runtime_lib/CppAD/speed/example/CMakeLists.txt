# -----------------------------------------------------------------------------
# CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-21 Bradley M. Bell
#
# CppAD is distributed under the terms of the
#              Eclipse Public License Version 2.0.
#
# This Source Code may also be made available under the following
# Secondary License when the conditions for such availability set forth
# in the Eclipse Public License, Version 2.0 are satisfied:
#       GNU General Public License, Version 2.0 or later.
# -----------------------------------------------------------------------------
# Build the speed/cppad directory tests
# Inherit build type from ../CMakeList.txt

# add_executable(<name> [WIN32] [MACOSX_BUNDLE] [EXCLUDE_FROM_ALL]
#                 source1 source2 ... sourceN
# )
SET(source_list example.cpp
    det_by_lu.cpp
    det_by_minor.cpp
    det_of_minor.cpp
    elapsed_seconds.cpp
    mat_sum_sq.cpp
    ode_evaluate.cpp
    sparse_hes_fun.cpp
    sparse_jac_fun.cpp
    speed_test.cpp
    time_test.cpp
)
set_compile_flags( speed_example "${cppad_debug_which}" "${source_list}" )
#
ADD_EXECUTABLE( speed_example EXCLUDE_FROM_ALL ${source_list} )

# Add the check_speed_example target
ADD_CUSTOM_TARGET(check_speed_example
    speed_example
    DEPENDS speed_example speed_src
)
MESSAGE(STATUS "make check_speed_example: available")
#
#
ADD_EXECUTABLE( speed_program EXCLUDE_FROM_ALL speed_program.cpp )

# Add the check_speed_program target
ADD_CUSTOM_TARGET(check_speed_program speed_program DEPENDS speed_program )
MESSAGE(STATUS "make check_speed_program: available")
#

# Change check depends in parent environment
add_to_list(check_speed_depends check_speed_example)
SET(check_speed_depends "${check_speed_depends}" PARENT_SCOPE)
