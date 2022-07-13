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
ELSE( cppad_has_eigen )
    SET(eigen_sources "")
ENDIF( cppad_has_eigen )
#
# BEGIN_SORT_THIS_LINE_PLUS_2
SET(source_list
    atomic_sparsity.cpp
    atomic_two.cpp
    base2ad.cpp
    for_sparse_hes.cpp
    for_sparse_jac.cpp
    forward.cpp
    get_started.cpp
    mat_mul.cpp
    norm_sq.cpp
    reciprocal.cpp
    reverse.cpp
    rev_sparse_hes.cpp
    rev_sparse_jac.cpp
    set_sparsity.cpp
    tangent.cpp
)
# END_SORT_THIS_LINE_MINUS_2

set_compile_flags(test_more_deprecated_atomic_two
    "${cppad_debug_which}" "${source_list}"
)
#
ADD_EXECUTABLE(test_more_deprecated_atomic_two
    EXCLUDE_FROM_ALL ${source_list}
)
#
# Add the check_test_more_deprecated_atomic_two target
ADD_CUSTOM_TARGET(check_test_more_deprecated_atomic_two
    test_more_deprecated_atomic_two
    DEPENDS test_more_deprecated_atomic_two
)
MESSAGE(STATUS "make check_test_more_deprecated_atomic_two: available")
#
# add to check check_test_more_deprecated_depends in parent environment
add_to_list(check_test_more_deprecated_depends
    check_test_more_deprecated_atomic_two
)
SET(check_test_more_deprecated_depends
    "${check_test_more_deprecated_depends}" PARENT_SCOPE
)
