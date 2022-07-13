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
# eigen_sources and CPPAD_HAS_EIGEN
IF( cppad_has_eigen )
    SET(eigen_sources eigen_mat_inv.cpp eigen_cholesky.cpp eigen_mat_mul.cpp)
ELSE( cppad_has_eigen )
    SET(eigen_sources "")
ENDIF( cppad_has_eigen )
#
# BEGIN_SORT_THIS_LINE_PLUS_2
SET(source_list
    ${eigen_sources}
    atomic_two.cpp
)
# END_SORT_THIS_LINE_MINUS_2

set_compile_flags( example_atomic_two "${cppad_debug_which}" "${source_list}" )
#
ADD_EXECUTABLE(example_atomic_two EXCLUDE_FROM_ALL ${source_list})
#
# Add the check_example_atomic_two target
ADD_CUSTOM_TARGET(check_example_atomic_two
    example_atomic_two
    DEPENDS example_atomic_two
)
MESSAGE(STATUS "make check_example_atomic_two: available")
#
# add to check check_example_depends in parent environment
add_to_list(check_example_depends check_example_atomic_two)
SET(check_example_depends "${check_example_depends}" PARENT_SCOPE)
