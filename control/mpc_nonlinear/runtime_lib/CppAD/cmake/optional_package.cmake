# -----------------------------------------------------------------------------
# CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-17 Bradley M. Bell
#
# CppAD is distributed under the terms of the
#              Eclipse Public License Version 2.0.
#
# This Source Code may also be made available under the following
# Secondary License when the conditions for such availability set forth
# in the Eclipse Public License, Version 2.0 are satisfied:
#       GNU General Public License, Version 2.0 or later.
# -----------------------------------------------------------------------------
# optional_package(package system_include description)
#
# ${package}_prefix: (out)
# is a PATH variable that holds the install prefix for this optional package.
#
# cppad_has_${package}: (out)
# is 1 if ${package}_prefix is set by the cmake command line (or gui),
# and 0 otherwise.
#
# system_include: (in)
# If this is true, the include files for this package should be treated as
# system files (no warnings).
#
# description: (in)
# is a description for the install prefix for this optional package.
#
# If ${package}_prefix is not set by the cmake command line (or gui),
# it is set to the default value NOTFOUND.
#
# If ${package}_prefix is set by the cmake command line, the following is done:
# 1. All the valid include subdirectories are added using INCLUDE_DIRECTORIES.
# 2. All the valid library subdirectories are added using LINK_DIRECTORIES.
# The set of valid include and library directories are determined by
# cmake_install_includedirs and cmakd_install_libdirs respectively.
#
# description: (in)
#
MACRO(optional_package package system_include description)
    SET(prefix_variable ${package}_prefix)
    SET(cppad_has_${package} 0)
    SET(${prefix_variable} NOTFOUND CACHE PATH "${description}")
    SET(prefix ${${prefix_variable}} )
    MESSAGE(STATUS "${prefix_variable} = ${prefix}")
    IF ( prefix )
        SET(cppad_has_${package} 1)
        #
        # List of preprocessor include file search directories
        FOREACH(dir ${cmake_install_includedirs})
            IF(IS_DIRECTORY ${prefix}/${dir} )
                IF( ${system_include} )
                    INCLUDE_DIRECTORIES( SYSTEM ${prefix}/${dir} )
                    MESSAGE(STATUS "    Found SYSTEM ${prefix}/${dir}")
                ELSE( ${system_include} )
                    INCLUDE_DIRECTORIES( ${prefix}/${dir} )
                    MESSAGE(STATUS "    Found ${prefix}/${dir}")
                ENDIF( ${system_include} )
            ENDIF(IS_DIRECTORY ${prefix}/${dir} )
        ENDFOREACH(dir)
        # Paths in which the linker will search for libraries,
        # only applies to targets created after it is called
        FOREACH(dir ${cmake_install_libdirs})
            IF(IS_DIRECTORY ${prefix}/${dir} )
                LINK_DIRECTORIES( ${prefix}/${dir} )
                MESSAGE(STATUS "    Found ${prefix}/${dir}")
            ENDIF(IS_DIRECTORY ${prefix}/${dir} )
        ENDFOREACH(dir)
    ENDIF ( prefix )
ENDMACRO(optional_package)
