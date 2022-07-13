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
# Build the introduction/exp_apx directory tests

# BEGIN_SORT_THIS_LINE_PLUS_2
SET(source_list
    exp_2.cpp
    exp_2_cppad.cpp
    exp_2_for0.cpp
    exp_2_for1.cpp
    exp_2_for2.cpp
    exp_2.hpp
    exp_2_rev1.cpp
    exp_2_rev2.cpp
    exp_eps.cpp
    exp_eps_cppad.cpp
    exp_eps_for0.cpp
    exp_eps_for1.cpp
    exp_eps_for2.cpp
    exp_eps.hpp
    exp_eps_rev1.cpp
    exp_eps_rev2.cpp
    introduction.cpp
)
# END_SORT_THIS_LINE_MINUS_2

set_compile_flags( introduction "${cppad_debug_which}" "${source_list}" )
#
ADD_EXECUTABLE( introduction EXCLUDE_FROM_ALL ${source_list} )

# Add this check target
ADD_CUSTOM_TARGET(check_introduction
    introduction
    DEPENDS introduction
)
MESSAGE(STATUS "make check_introduction: available")

# Change check depends in parent environment
add_to_list(check_depends check_introduction)
SET(check_depends "${check_depends}" PARENT_SCOPE)
