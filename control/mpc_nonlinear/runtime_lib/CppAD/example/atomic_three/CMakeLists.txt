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
#
# BEGIN_SORT_THIS_LINE_PLUS_2
SET(source_list
    atomic_three.cpp
    base2ad.cpp
    dynamic.cpp
    forward.cpp
    get_started.cpp
    hes_sparsity.cpp
    jac_sparsity.cpp
    mat_mul.cpp
    norm_sq.cpp
    reciprocal.cpp
    rev_depend.cpp
    reverse.cpp
    tangent.cpp
    vector_op.cpp
)
# END_SORT_THIS_LINE_MINUS_2

set_compile_flags(example_atomic_three "${cppad_debug_which}" "${source_list}" )
#
ADD_EXECUTABLE(example_atomic_three EXCLUDE_FROM_ALL ${source_list})
#
# Add the check_example_atomic_three target
ADD_CUSTOM_TARGET(check_example_atomic_three
    example_atomic_three
    DEPENDS example_atomic_three
)
MESSAGE(STATUS "make check_example_atomic_three: available")
#
# add to check check_example_depends in parent environment
add_to_list(check_example_depends check_example_atomic_three)
SET(check_example_depends "${check_example_depends}" PARENT_SCOPE)
