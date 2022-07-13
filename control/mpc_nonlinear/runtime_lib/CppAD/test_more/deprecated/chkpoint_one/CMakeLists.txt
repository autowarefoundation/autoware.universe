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
SET(source_list
    chkpoint_one.cpp
    extended_ode.cpp
    get_started.cpp
    mul_level.cpp
    ode.cpp
)
# END_SORT_THIS_LINE_MINUS_2

set_compile_flags(test_more_deprecated_chkpoint_one
    "${cppad_debug_which}" "${source_list}"
)
#
ADD_EXECUTABLE(test_more_deprecated_chkpoint_one
    EXCLUDE_FROM_ALL ${source_list}
)
#
# Add the check_test_more_deprecated_chkpoint_one target
ADD_CUSTOM_TARGET(check_test_more_deprecated_chkpoint_one
    test_more_deprecated_chkpoint_one
    DEPENDS test_more_deprecated_chkpoint_one
)
MESSAGE(STATUS "make check_test_more_deprecated_chkpoint_one: available")
#
# add to check check_test_more_deprecated_depends in parent environment
add_to_list(check_test_more_deprecated_depends
    check_test_more_deprecated_chkpoint_one
)
SET(check_test_more_deprecated_depends
    "${check_test_more_deprecated_depends}" PARENT_SCOPE
)
