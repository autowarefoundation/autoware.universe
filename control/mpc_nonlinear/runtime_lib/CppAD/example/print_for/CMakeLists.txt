# -----------------------------------------------------------------------------
# CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-17 Bradley M. Bell
#
# CppAD is distributed under the terms of the
#              Eclipse Public License Version 2.0.
#
# This Source Code may also be made available under the following
# Secondary License when the conditions for such availability set forth
# in the Eclipse Public License, Version 2.0 are satisfied:
#       GNU General Public License, Version 2.0 or later.
# -----------------------------------------------------------------------------
# Build the example/print_for directory tests
#
SET(source_list print_for.cpp)
#
set_compile_flags( example_print_for "${cppad_debug_which}" "${source_list}" )
#
ADD_EXECUTABLE(example_print_for EXCLUDE_FROM_ALL ${source_list})

# check_example_print_for target
ADD_CUSTOM_TARGET(check_example_print_for
    example_print_for
    DEPENDS
    example_print_for
)
MESSAGE(STATUS "make check_example_print_for: available")

# Change check_example_depends in parent environment
add_to_list(check_example_depends check_example_print_for)
SET(check_example_depends "${check_example_depends}" PARENT_SCOPE)
