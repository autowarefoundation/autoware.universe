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
# pkgconfig_info(name system coin_or)
#
# Inputs:
# name:    is the name of the package config file without the .pc extension
# system:  if true (false) include directories will (will not) be
#          treated like a system directory; i.e., no warnings.
# coin_or: if true (false) the string '/coin-or' at the end of an include
#          include directory is (is not) removed because CppAD explicitly
#           includes it in include commands.
#
# Outputs:
# ${name}_LIBRARIES:     list of libraries necessary to use this package.
#
# Side Effects:
# Include directors required to use package are passed to INCLUDE_DIRECTORIES.
# Link directories requried to link ${name}_LIBRARIES are passed to
# LINK_DIRECTORIES, and the SYSTEM flag is inclued if system is TRUE.
#
# Assumptions:
# FIND_PACKAGE(PkgConfig) was successful
# It is a fatal error if ${name}.pc is not in PKG_CONFIG_PATH
MACRO(pkgconfig_info name system coin_or)
    IF( NOT PKG_CONFIG_FOUND )
        FIND_PACKAGE(PkgConfig REQUIRED)
    ENDIF( NOT PKG_CONFIG_FOUND )
    pkg_check_modules( ${name} QUIET ${name} )
    IF( ${name}_FOUND )
        MESSAGE(STATUS "Found ${name}.pc file")
    ELSE( ${name}_FOUND )
        MESSAGE(FATAL_ERROR
            "Cannot find ${name}.pc or one of the packages it requires."
            " PKG_CONFIG_PATH=$ENV{PKG_CONFIG_PATH}"
        )
    ENDIF( ${name}_FOUND )
    #
    IF( ${coin_or} )
        STRING(REPLACE "/coin-or" "" include_dirs "${${name}_INCLUDE_DIRS}" )
        SET(${name}_INCLUDE_DIRS "${include_dirs}")
    ENDIF( ${coin_or} )
    #
    # INLCUDE_DIRECTORIES
    IF( ${system} )
        INCLUDE_DIRECTORIES( SYSTEM ${include_dirs} )
    ELSE( ${system}  )
        INCLUDE_DIRECTORIES( ${include_dirs} )
    ENDIF( ${system} )
    #
    # LINK_DIRECTORIES
    LINK_DIRECTORIES( ${${name}_LIBRARY_DIRS} )
ENDMACRO(pkgconfig_info)
