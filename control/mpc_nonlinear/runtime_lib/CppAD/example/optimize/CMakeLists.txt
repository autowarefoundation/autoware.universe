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
# Build the example/optimize directory tests
#
#
# BEGIN_SORT_THIS_LINE_PLUS_2
SET(source_list
    compare_op.cpp
    conditional_skip.cpp
    cumulative_sum.cpp
    forward_active.cpp
    nest_conditional.cpp
    optimize.cpp
    optimize_twice.cpp
    print_for.cpp
    reverse_active.cpp
)
# END_SORT_THIS_LINE_MINUS_2

set_compile_flags( example_optimize "${cppad_debug_which}" "${source_list}" )
#
ADD_EXECUTABLE(example_optimize EXCLUDE_FROM_ALL ${source_list})

# Add the check_example_optimize target
ADD_CUSTOM_TARGET(check_example_optimize
    example_optimize
    DEPENDS example_optimize
)
MESSAGE(STATUS "make check_example_optimize: available")

# add to check check_example_depends in parent environment
add_to_list(check_example_depends check_example_optimize)
SET(check_example_depends "${check_example_depends}" PARENT_SCOPE)
