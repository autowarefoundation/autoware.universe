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
# Build the example/ipopt_solve directory tests

# Local link directories to search, as determined by pkg-config for ipopt
LINK_DIRECTORIES( ${ipopt_LIBRARY_DIRS} )
#
# BEGIN_SORT_THIS_LINE_PLUS_2
SET(source_list
    get_started.cpp
    ipopt_solve.cpp
    ode_inverse.cpp
    retape.cpp
)
# END_SORT_THIS_LINE_MINUS_2

set_compile_flags( example_ipopt_solve "${cppad_debug_which}" "${source_list}" )
#
ADD_EXECUTABLE(example_ipopt_solve EXCLUDE_FROM_ALL ${source_list})
#
# libraries to be linked into the specified target,
# as determined by pkg-config for ipopt
TARGET_LINK_LIBRARIES(example_ipopt_solve
    ${cppad_lib}
    ${ipopt_LIBRARIES}
    ${colpack_libs}
)

# Add the check_example_ipopt_solve target
ADD_CUSTOM_TARGET(check_example_ipopt_solve
    example_ipopt_solve
    DEPENDS example_ipopt_solve
)
MESSAGE(STATUS "make check_example_ipopt_solve: available")

# Change check_example_depends in parent environment
add_to_list(check_example_depends check_example_ipopt_solve)
SET(check_example_depends "${check_example_depends}" PARENT_SCOPE)
