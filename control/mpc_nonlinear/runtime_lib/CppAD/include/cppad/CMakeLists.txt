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
# Configure the CppAD include file directory
# -----------------------------------------------------------------------------
# check_match
MACRO(check_match match_variable match_constant output_variable)
    STRING(COMPARE EQUAL ${${match_variable}} ${match_constant} match_flag )
    IF( match_flag )
        SET(${output_variable} 1)
    ELSE( match_flag )
        SET(${output_variable} 0)
    ENDIF( match_flag )
    print_variable(${output_variable})
ENDMACRO(check_match)
# -----------------------------------------------------------------------------
# CMAKE_REQUIRED_name
SET(CMAKE_REQUIRED_DEFINITIONS "")
SET(CMAKE_REQUIRED_FLAGS       "")
SET(CMAKE_REQUIRED_INCLUDES    "")
SET(CMAKE_REQUIRED_LIBRARIES   "")
# -----------------------------------------------------------------------------
# compiler_has_conversion_warn
SET( clang_or_gnu 0 )
IF( "${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang" )
    SET(clang_or_gnu 1)
ENDIF( "${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang" )
IF( "${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU" )
    SET(clang_or_gnu 1)
ENDIF( "${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU" )
IF( clang_or_gnu )
    SET(CMAKE_REQUIRED_FLAGS
        "${cppad_cxx_flags} -Wfloat-conversion -Wconversion -Werror"
    )
    #
    SET(source "int main(void) { return 0; }")
    compile_source_test("${source}" compiler_has_conversion_warn )
    #
    SET(CMAKE_REQUIRED_FLAGS "")
ELSE( clang_or_gnu )
    SET( compiler_has_conversion_warn 0 )
ENDIF( clang_or_gnu )
# -----------------------------------------------------------------------------
# cppad_boostvector, cppad_cppadvector, cppad_eigenvector, cppad_stdvector
#
check_match(cppad_testvector boost cppad_boostvector)
check_match(cppad_testvector cppad cppad_cppadvector)
check_match(cppad_testvector eigen cppad_eigenvector)
check_match(cppad_testvector std   cppad_stdvector)
IF( NOT cppad_boostvector )
IF( NOT cppad_cppadvector )
IF( NOT cppad_eigenvector )
IF( NOT cppad_stdvector )
MESSAGE(FATAL_ERROR
"cppad_testvector not one of following: boost, cppad, eigen, std."
"This should have been found earlier, please report this as a bug."
)
ENDIF( NOT cppad_stdvector )
ENDIF( NOT cppad_eigenvector )
ENDIF( NOT cppad_cppadvector )
ENDIF( NOT cppad_boostvector )
#
IF( cppad_boostvector )
    # FIND_PACKAGE(Boost) done by ../CMakeLists.txt
    IF( NOT Boost_FOUND )
        MESSAGE(FATAL_ERROR
"cppad_testvector == boost but cannot find boost include files"
        )
    ENDIF( NOT Boost_FOUND )
ENDIF( cppad_boostvector )
#
IF( cppad_eigenvector )
    IF( NOT include_eigen )
        MESSAGE(FATAL_ERROR
"cppad_testvector == eigen but eigen_prefix is not specified"
        )
    ENDIF( NOT include_eigen )
ENDIF( cppad_eigenvector )
# -----------------------------------------------------------------------------
# cppad_has_gettimeofday
#
SET(source "
# include<sys/time.h>
int main(void)
{   struct timeval time;
    gettimeofday(&time, 0);
    return 0;
}"
)
compile_source_test("${source}" cppad_has_gettimeofday)
# -----------------------------------------------------------------------------
# Warn user of the following types are signed:
#   cppad_tape_addr_type, cppad_tape_id_type
FOREACH(cmake_var cppad_tape_id_type cppad_tape_addr_type )
    SET(source "
# include <limits>
# include <cstddef>
int main(void)
{   static_assert(
        ! std::numeric_limits<${${cmake_var}}>::is_signed ,
        \"${cmake_var} is a signed type\"
    );
    return 0;
}
"
    )
    compile_source_test("${source}" ${cmake_var}_is_unsigned)
    IF( NOT ${${cmake_var}_is_unsigned} )
        MESSAGE(STATUS
"Warning: using a signed type for ${cmake_var} is for CppAD developers only !"
        )
    ENDIF( NOT ${${cmake_var}_is_unsigned} )
ENDFOREACH( cmake_var )
# -----------------------------------------------------------------------------
# cppad_has_mkstemp
#
SET(source "
# include <stdlib.h>
# include <unistd.h>
int main(void)
{
    char pattern[] = \"/tmp/fileXXXXXX\";
    int fd = mkstemp(pattern);
    return 0;
}
" )
compile_source_test("${source}" cppad_has_mkstemp )
# -----------------------------------------------------------------------------
# cppad_has_tmpname_s
#
SET(source "
# include <stdio.h>
int main(void)
{   char filename[L_tmpnam_s ];
    if( tmpnam_s(filename, L_tmpnam_s ) != 0 )
        return 1;
    return 0;
}
" )
compile_source_test("${source}" cppad_has_tmpnam_s )
# -----------------------------------------------------------------------------
# configure.hpp
CONFIGURE_FILE(
    ${CMAKE_CURRENT_SOURCE_DIR}/configure.hpp.in
    ${CMAKE_CURRENT_SOURCE_DIR}/configure.hpp
)
# -----------------------------------------------------------------------------
