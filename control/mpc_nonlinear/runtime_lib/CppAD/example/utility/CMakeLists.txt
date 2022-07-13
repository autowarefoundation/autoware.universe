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
#
# BEGIN_SORT_THIS_LINE_PLUS_2
SET(source_list
    check_numeric_type.cpp
    check_simple_vector.cpp
    cppad_vector.cpp
    error_handler.cpp
    index_sort.cpp
    lu_factor.cpp
    lu_invert.cpp
    lu_solve.cpp
    nan.cpp
    near_equal.cpp
    ode_err_control.cpp
    ode_err_maxabs.cpp
    ode_gear.cpp
    ode_gear_control.cpp
    poly.cpp
    pow_int.cpp
    romberg_mul.cpp
    romberg_one.cpp
    rosen_34.cpp
    runge45_1.cpp
    runge_45.cpp
    set_union.cpp
    simple_vector.cpp
    sparse_rc.cpp
    sparse_rcv.cpp
    thread_alloc.cpp
    to_string.cpp
    utility.cpp
    vector_bool.cpp
)
# END_SORT_THIS_LINE_MINUS_2

set_compile_flags( example_utility "${cppad_debug_which}" "${source_list}" )
#
ADD_EXECUTABLE(example_utility EXCLUDE_FROM_ALL ${source_list})
#
# Add the check_example_utility target
ADD_CUSTOM_TARGET(check_example_utility
    example_utility
    DEPENDS example_utility
)
MESSAGE(STATUS "make check_example_utility: available")

# add to check check_example_depends in parent environment
add_to_list(check_example_depends check_example_utility)
SET(check_example_depends "${check_example_depends}" PARENT_SCOPE)
