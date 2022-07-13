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
# compile_source_test(source variable)
#
# source: (in)
# contains the source for the program that will be compiled and linked.
#
# variable: (out)
# This variable must not be defined when this macro is called.
# Upon return, the value of this variable is 1 (0) if the program compiles
# and links (does not compile and link).
#
# CMAKE_REQUIRED_name (in)
# For name equal to DEFINITIONS, INCLUDES, LIBRARIES, FLAGS, the variable
# CMAKE_REQUIRED_name is an input to routine; see CHECK_CXX_SOURCE_COMPILES
# documentation.
#
MACRO(compile_source_test source variable)
    #
    # check that variable is not yet defined
    IF( DEFINED ${variable} )
        MESSAGE(FATAL_ERROR
            "compile_source_test: ${variable} is defined before expected"
        )
    ENDIF( DEFINED ${variable} )
    #
    IF( DEFINED compiles_source_test_result)
        UNSET(compiles_source_test_result)
    ENDIF( DEFINED compiles_source_test_result )
    #
    # check that source codee compiles
    CHECK_CXX_SOURCE_COMPILES("${source}" compiles_source_test_result )
    #
    # change result varialbe to 0 (1) for fail (succeed).
    IF( compiles_source_test_result )
        SET(${variable} 1)
    ELSE( compiles_source_test_result )
        SET(${variable} 0)
    ENDIF( compiles_source_test_result )
    #
    # check that varialbe is defined
    IF( NOT DEFINED ${variable} )
        MESSAGE(FATAL_ERROR
            "compile_source_test: error in CMake script."
        )
    ENDIF( NOT DEFINED ${variable} )
    #
    MESSAGE(STATUS "${variable} = ${${variable}}" )
ENDMACRO( compile_source_test )
