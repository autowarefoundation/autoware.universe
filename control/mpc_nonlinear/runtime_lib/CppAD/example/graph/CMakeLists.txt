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
    add_op.cpp
    atom_op.cpp
    azmul_op.cpp
    cexp_op.cpp
    comp_op.cpp
    discrete_op.cpp
    div_op.cpp
    graph.cpp
    mul_op.cpp
    pow_op.cpp
    print_graph.cpp
    print_op.cpp
    sub_op.cpp
    sum_op.cpp
    switch_var_dyn.cpp
    unary_op.cpp
)
# END_SORT_THIS_LINE_MINUS_2

set_compile_flags(example_graph "${cppad_debug_which}" "${source_list}" )
#
ADD_EXECUTABLE(example_graph EXCLUDE_FROM_ALL ${source_list})

# List of libraries to be linked into the specified target
TARGET_LINK_LIBRARIES(example_graph
    ${cppad_lib}
    ${colpack_libs}
)
#
# Add the check_example_graph target
ADD_CUSTOM_TARGET(check_example_graph
    example_graph
    DEPENDS example_graph
)
MESSAGE(STATUS "make check_example_graph: available")
#
# add to check check_example_depends in parent environment
add_to_list(check_example_depends check_example_graph)
SET(check_example_depends "${check_example_depends}" PARENT_SCOPE)
