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
# Name for this text
SET(name cppad_for_tmb)
#
# Add extra compiler flags to cppad_cxx_flags so set_compile_flags uses them.
# This change only affects the cppad_for_tmb tests (NO PARENT_SCOPE).
SET(cppad_cxx_flags "${cppad_cxx_flags} ${OpenMP_CXX_FLAGS} -DCPPAD_FOR_TMB")
#
# BEGIN_SORT_THIS_LINE_PLUS_2
SET(source_list
    ${name}.cpp
    implicit_ctor.cpp
    multi_atomic_two.cpp
    multi_atomic_three.cpp
    multi_chkpoint_one.cpp
    multi_chkpoint_two.cpp
    perfer_reverse.cpp
)
# END_SORT_THIS_LINE_MINUS_2

set_compile_flags(
    test_more_${name} "${cppad_debug_which}" "${source_list}"
)
#
# now that we have the properties, add the executable
ADD_EXECUTABLE( test_more_${name} EXCLUDE_FROM_ALL ${source_list} )
#
# Extra flags used by linker for openmp support
SET(CMAKE_EXE_LINKER_FLAGS ${OpenMP_CXX_FLAGS} )
#
# check_test_more_${name} target
ADD_CUSTOM_TARGET(
    check_test_more_${name} test_more_${name} DEPENDS test_more_${name}
)
MESSAGE(STATUS "make check_test_more_${name}: available")

# Add check_test_more_${name} to check depends in parent environment
add_to_list(check_test_more_depends check_test_more_${name})
SET(check_test_more_depends "${check_test_more_depends}" PARENT_SCOPE)
