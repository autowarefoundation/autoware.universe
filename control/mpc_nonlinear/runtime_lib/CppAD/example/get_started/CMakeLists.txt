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
# Build the example/get_started directory tests
#
SET(source_list get_started.cpp)
#
set_compile_flags( example_get_started "${cppad_debug_which}" "${source_list}" )
#
ADD_EXECUTABLE(example_get_started EXCLUDE_FROM_ALL ${source_list})
#
# Add the check for this target
ADD_CUSTOM_TARGET(check_example_get_started
    example_get_started
    DEPENDS example_get_started
)
MESSAGE(STATUS "make check_example_get_started: available")
#
# Change check depends in parent environment
add_to_list(check_example_depends check_example_get_started)
SET(check_example_depends "${check_example_depends}" PARENT_SCOPE)
