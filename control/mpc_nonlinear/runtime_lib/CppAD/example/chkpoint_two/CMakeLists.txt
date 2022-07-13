# -----------------------------------------------------------------------------
# CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-19 Bradley M. Bell
#
# CppAD is distributed under the terms of the
#              Eclipse Public License Version 2.0.
#
# This Source Code may also be made available under the following
# Secondary License when the conditions for such availability set forth
# in the Eclipse Public License, Version 2.0 are satisfied:
#       GNU General Public License, Version 2.0 or later.
# -----------------------------------------------------------------------------
#
# BEGIN_SORT_THIS_LINE_PLUS_2
SET(source_list
    base2ad.cpp
    chkpoint_two.cpp
    compare.cpp
    dynamic.cpp
    get_started.cpp
    ode.cpp
)
# END_SORT_THIS_LINE_MINUS_2

set_compile_flags( example_chkpoint_two "${cppad_debug_which}" "${source_list}" )
#
ADD_EXECUTABLE(example_chkpoint_two EXCLUDE_FROM_ALL ${source_list})
#
# Add the check_example_chkpoint_two target
ADD_CUSTOM_TARGET(check_example_chkpoint_two
    example_chkpoint_two
    DEPENDS example_chkpoint_two
)
MESSAGE(STATUS "make check_example_chkpoint_two: available")
#
# add to check check_example_depends in parent environment
add_to_list(check_example_depends check_example_chkpoint_two)
SET(check_example_depends "${check_example_depends}" PARENT_SCOPE)
