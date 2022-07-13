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
SET(source_list sparse.cpp
    colpack_hes.cpp
    colpack_hessian.cpp
    colpack_jac.cpp
    colpack_jacobian.cpp
    conj_grad.cpp
    dependency.cpp
    for_hes_sparsity.cpp
    for_jac_sparsity.cpp
    for_sparse_hes.cpp
    for_sparse_jac.cpp
    rc_sparsity.cpp
    rev_hes_sparsity.cpp
    rev_jac_sparsity.cpp
    rev_sparse_hes.cpp
    rev_sparse_jac.cpp
    sparse_hes.cpp
    sparse_hessian.cpp
    sparse_jac_for.cpp
    sparse_jac_rev.cpp
    sparse_jacobian.cpp
    sparse_sub_hes.cpp
    sparsity_sub.cpp
    sub_sparse_hes.cpp
    subgraph_hes2jac.cpp
    subgraph_jac_rev.cpp
    subgraph_reverse.cpp
    subgraph_sparsity.cpp
)
# END_SORT_THIS_LINE_MINUS_2

IF( cppad_has_eigen )
    SET(source_list sparse2eigen.cpp ${source_list} )
ENDIF( cppad_has_eigen )

set_compile_flags( example_sparse "${cppad_debug_which}" "${source_list}" )
#
ADD_EXECUTABLE(example_sparse EXCLUDE_FROM_ALL ${source_list})

# List of libraries to be linked into the specified target
TARGET_LINK_LIBRARIES(example_sparse
    ${cppad_lib}
    ${colpack_libs}
)

# Add the check_example_sparse target
ADD_CUSTOM_TARGET(check_example_sparse
    example_sparse
    DEPENDS example_sparse
)
MESSAGE(STATUS "make check_example_sparse: available")

# add to check_example_depends in parent environment
add_to_list(check_example_depends check_example_sparse)
SET(check_example_depends "${check_example_depends}" PARENT_SCOPE)
