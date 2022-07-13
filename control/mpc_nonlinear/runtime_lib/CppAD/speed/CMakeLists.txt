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
SET(check_speed_depends "")

# The CMakeLists.txt file in the specified source directory is processed
# before the current input file continues beyond this command.
# add_subdirectory(source_dir [binary_dir] [EXCLUDE_FROM_ALL])
ADD_SUBDIRECTORY(src)
ADD_SUBDIRECTORY(cppad)
ADD_SUBDIRECTORY(double)
ADD_SUBDIRECTORY(example)
ADD_SUBDIRECTORY(xpackage)
IF ( cppad_profile_flag )
    ADD_SUBDIRECTORY(profile)
ENDIF ( cppad_profile_flag )
#
IF( cppad_has_adolc )
    ADD_SUBDIRECTORY(adolc)
ENDIF( cppad_has_adolc )
IF( cppad_has_fadbad )
    ADD_SUBDIRECTORY(fadbad)
ENDIF( cppad_has_fadbad )
IF( cppad_has_cppadcg )
    ADD_SUBDIRECTORY(cppadcg)
ENDIF( cppad_has_cppadcg )
IF( cppad_has_sacado )
    ADD_SUBDIRECTORY(sacado)
ENDIF( cppad_has_sacado )

# check_speed
ADD_CUSTOM_TARGET(check_speed DEPENDS ${check_speed_depends} )
MESSAGE(STATUS "make check_speed: available")

# Change check depends in parent environment
add_to_list(check_depends check_speed)
SET(check_depends "${check_depends}" PARENT_SCOPE)
