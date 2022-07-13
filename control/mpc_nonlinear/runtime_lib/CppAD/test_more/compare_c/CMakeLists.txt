# -----------------------------------------------------------------------------
# CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-20 Bradley M. Bell
#
# CppAD is distributed under the terms of the
#              Eclipse Public License Version 2.0.
#
# This Source Code may also be made available under the following
# Secondary License when the conditions for such availability set forth
# in the Eclipse Public License, Version 2.0 are satisfied:
#       GNU General Public License, Version 2.0 or later.
# -----------------------------------------------------------------------------
# $begin compare_c$$ $newlinech #$$
# $spell
#   det
#   cpp
# $$
#
# $section Compare Speed of C and C++$$
# $index compare, speed C and C++$$
# $index speed, compare C and C++$$
# $index C, compare speed with C++$$
# $index C++, compare speed with C$$
#
# $head Syntax$$
# $codei%test_more/compare_c/det_by_minor_c
# %$$
# $codei test_more/compare_c/det_by_minor_cpp$$
#
# $head Purpose$$
# Compares the speed of the exact same source code compiled
# using C versus C++.
#
# $childtable%
#   test_more/compare_c/det_by_minor.c
# %$$
#
# $end
# -----------------------------------------------------------------------------
#
# use cppad_debug_which to determine build type
IF( "${cppad_debug_which}" STREQUAL debug_all )
    SET(CMAKE_BUILD_TYPE DEBUG)
    SET(all_cxx_flags "${cppad_cxx_flags} ${CMAKE_CXX_FLAGSS_DEBUG}")
ELSEIF( "${cppad_debug_which}" STREQUAL debug_odd )
    SET(CMAKE_BUILD_TYPE DEBUG)
    SET(all_cxx_flags "${cppad_cxx_flags} ${CMAKE_CXX_FLAGSS_DEBUG}")
ELSE( "${cppad_debug_which}" STREQUAL debug_odd )
    SET(CMAKE_BUILD_TYPE RELEASE)
    SET(all_cxx_flags "${cppad_cxx_flags} ${CMAKE_CXX_FLAGSS_RELEASE}")
ENDIF( "${cppad_debug_which}" STREQUAL debug_all )
#
# Loop though the C and C++ compilers
FOREACH( com c cpp )
    # Copy a file to another location and modify its contents.
    # configure_file(InputFile OutputFile [COPYONLY] [ESCAPE_QUOTES] [@ONLY])
    SET( source det_by_minor_${com}.${com} )
    CONFIGURE_FILE(
        ${CMAKE_CURRENT_SOURCE_DIR}/det_by_minor.c
        ${CMAKE_CURRENT_BINARY_DIR}/${source}
        COPYONLY
    )
    ADD_EXECUTABLE( det_by_minor_${com} EXCLUDE_FROM_ALL ${source})
    #
    IF( ${com} STREQUAL cpp )
        # cppad_cxx_flags are C++ compiler flags and may not be valid for C
        SET_TARGET_PROPERTIES(
            det_by_minor_${com} PROPERTIES COMPILE_FLAGS "${all_cxx_flags}"
        )
    ENDIF( ${com} STREQUAL cpp )
    #
    # Add target the executes this program
    ADD_CUSTOM_TARGET(check_det_by_minor_${com}
        det_by_minor_${com}
        DEPENDS det_by_minor_${com}
    )
    MESSAGE(STATUS "make check_det_by_minor_${com}: available")

ENDFOREACH(com)

# check_test_more_compare_c target
ADD_CUSTOM_TARGET( check_test_more_compare_c
    DEPENDS check_det_by_minor_c check_det_by_minor_cpp
)
MESSAGE(STATUS "make check_test_more_compare_c: available")

# Add check_test_more_compare_c to check depends in parent environment
add_to_list(check_test_more_depends check_test_more_compare_c)
SET(check_test_more_depends "${check_test_more_depends}" PARENT_SCOPE)
