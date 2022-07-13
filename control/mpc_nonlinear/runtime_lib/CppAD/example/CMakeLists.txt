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
# Build the example directory tests
# Inherit environment from ../CMakeList.txt

# initialize check_example_depends
SET(check_example_depends "")

# abs_normal examples
ADD_SUBDIRECTORY(abs_normal)

# atomic examples
ADD_SUBDIRECTORY(atomic_two)
ADD_SUBDIRECTORY(atomic_three)

# checkpoint exmaples
ADD_SUBDIRECTORY(chkpoint_two)

# code_gen_fun examples
IF( cpapd_has_cpapdcg )
    ADD_SUBDIRECTORY(code_gen_fun)
ENDIF( cpapd_has_cpapdcg )

# graph examples
ADD_SUBDIRECTORY(graph)

# general examples
ADD_SUBDIRECTORY(general)

# get_started
ADD_SUBDIRECTORY(get_started)

# ipopt_solve examples
IF( cppad_has_ipopt)
    ADD_SUBDIRECTORY(ipopt_solve)
ENDIF( cppad_has_ipopt)

# Json AD graph examples
ADD_SUBDIRECTORY(json)

# multi_thread examples
ADD_SUBDIRECTORY(multi_thread)

# optimize examples
ADD_SUBDIRECTORY(optimize)

# print_for example
ADD_SUBDIRECTORY(print_for)

# sparse examples
ADD_SUBDIRECTORY(sparse)

# utility examples
ADD_SUBDIRECTORY(utility)

# Add the check_example target
ADD_CUSTOM_TARGET(check_example "" DEPENDS ${check_example_depends})
MESSAGE(STATUS "make check_example: available")

# Change check depends in parent environment
add_to_list(check_depends check_example)
SET(check_depends "${check_depends}" PARENT_SCOPE)
