# -----------------------------------------------------------------------------
# CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-22 Bradley M. Bell
#
# CppAD is distributed under the terms of the
#              Eclipse Public License Version 2.0.
#
# This Source Code may also be made available under the following
# Secondary License when the conditions for such availability set forth
# in the Eclipse Public License, Version 2.0 are satisfied:
#       GNU General Public License, Version 2.0 or later.
# -----------------------------------------------------------------------------
# Build the test_more/general tests

# set compiler flags for debug_rel.cpp and debug.cpp
IF( "${cppad_debug_which}" STREQUAL "debug_all" )
    SET(debug_flags    "${cppad_cxx_flags} ${CMAKE_CXX_FLAGS_DEBUG}")
    SET(release_flags  "${cppad_cxx_flags} ${CMAKE_CXX_FLAGS_DEBUG}")
ELSEIF( "${cppad_debug_which}" STREQUAL "debug_none" )
    SET(debug_flags    "${cppad_cxx_flags} ${CMAKE_CXX_FLAGS_RELEASE}")
    SET(release_flags  "${cppad_cxx_flags} ${CMAKE_CXX_FLAGS_RELEASE}")
ELSE( "${cppad_debug_which}" )
    SET(debug_flags    "${cppad_cxx_flags} ${CMAKE_CXX_FLAGS_DEBUG}")
    SET(release_flags  "${cppad_cxx_flags} ${CMAKE_CXX_FLAGS_RELEASE}")
ENDIF( "${cppad_debug_which}" STREQUAL "debug_all" )
#
SET_SOURCE_FILES_PROPERTIES(
    debug_rel.cpp debug.cpp PROPERTIES COMPILE_FLAGS
    "${debug_flags} -DCPPAD_DEBUG_AND_RELEASE"
)
#
SET_SOURCE_FILES_PROPERTIES(
    release.cpp PROPERTIES COMPILE_FLAGS
    "${release_flags} -DCPPAD_DEBUG_AND_RELEASE"
)

# now that we have the properties, add the executable
ADD_EXECUTABLE(test_more_debug_rel EXCLUDE_FROM_ALL
    debug_rel.cpp
    debug.cpp
    release.cpp
)

# Add the check_test_more_debug_rel target
ADD_CUSTOM_TARGET(check_test_more_debug_rel
    test_more_debug_rel
    DEPENDS
    test_more_debug_rel
)
MESSAGE(STATUS "make check_test_more_debug_rel: available")

# Change check depends in parent environment
add_to_list(check_test_more_depends check_test_more_debug_rel)
SET(check_test_more_depends "${check_test_more_depends}" PARENT_SCOPE)
