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
# Build the test_more/deprecated tests
#
SET(check_test_more_deprecated_depends "test_more_deprecated")
ADD_SUBDIRECTORY(atomic_two)
ADD_SUBDIRECTORY(chkpoint_one)
#
# BEGIN_SORT_THIS_LINE_PLUS_2
SET(source_list
    deprecated.cpp
    old_mat_mul.cpp
    old_reciprocal.cpp
    old_tan.cpp
    old_usead_1.cpp
    old_usead_2.cpp
    omp_alloc.cpp
    track_new_del.cpp
    zdouble.cpp
)
# END_SORT_THIS_LINE_MINUS_2

set_compile_flags( test_more_deprecated "${cppad_debug_which}" "${source_list}" )
#
ADD_EXECUTABLE(test_more_deprecated EXCLUDE_FROM_ALL ${source_list})
#
# Add the check_test_more_deprecated target
ADD_CUSTOM_TARGET(check_test_more_deprecated
    test_more_deprecated
    DEPENDS
    ${check_test_more_deprecated_depends}
)
MESSAGE(STATUS "make check_test_more_deprecated: available")
#
# Change check depends in parent environment
add_to_list(check_test_more_depends check_test_more_deprecated)
SET(check_test_more_depends "${check_test_more_depends}" PARENT_SCOPE)
