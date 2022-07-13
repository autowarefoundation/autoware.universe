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
# Build the test_more directory tests

# initialize check_depends
SET(check_test_more_depends "")

# deprecated tests
ADD_SUBDIRECTORY(deprecated)

# compare_c tests
ADD_SUBDIRECTORY(compare_c)

# debug_rel tests
IF( NOT "${CMAKE_GENERATOR}" STREQUAL "NMake Makefiles" )
    # Visual studio rejects mixing debug and release flags
    ADD_SUBDIRECTORY(debug_rel)
ENDIF( NOT "${CMAKE_GENERATOR}" STREQUAL "NMake Makefiles" )
#
# cppad_for_tmb tests
IF( OPENMP_FOUND )
    ADD_SUBDIRECTORY(cppad_for_tmb)
ENDIF( OPENMP_FOUND )

# general tests
ADD_SUBDIRECTORY(general)

# Add the check_example target
ADD_CUSTOM_TARGET(check_test_more "" DEPENDS ${check_test_more_depends})
MESSAGE(STATUS "make check_test_more: available")

# Change check depends in parent environment
add_to_list(check_depends check_test_more)
SET(check_depends "${check_depends}" PARENT_SCOPE)
