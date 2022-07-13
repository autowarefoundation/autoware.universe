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

# Initialize list of tests as empty
SET(check_cppad_ipopt_depends "")

# assert include_ipopt
IF ( NOT include_ipopt )
    MESSAGE(FATAL_ERROR
        "include_ipopt is not on the cmake command line"
    )
ENDIF ( NOT include_ipopt )

# The CMakeLists.txt file in the specified source directory is processed
# before the current input file continues beyond this command.
# add_subdirectory(source_dir [binary_dir] [EXCLUDE_FROM_ALL])
ADD_SUBDIRECTORY(src)
ADD_SUBDIRECTORY(example)
ADD_SUBDIRECTORY(speed)
ADD_SUBDIRECTORY(test)

# Add the check_cppad_ipopt target
ADD_CUSTOM_TARGET(check_cppad_ipopt DEPENDS ${check_cppad_ipopt_depends} )
MESSAGE(STATUS "make check_cppad_ipopt: available")

# Change check depends in parent environment
add_to_list(check_depends check_cppad_ipopt)
SET(check_depends "${check_depends}" PARENT_SCOPE)
