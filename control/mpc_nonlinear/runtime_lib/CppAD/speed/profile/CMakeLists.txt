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
# Build the speed/profile directory tests
# Inherit build type from ../CMakeList.txt

# Adds flags to the compiler command line for sources in the current directory
# and below. This command can be used to add any flags, but it was originally
# intended to add preprocessor definitions.
ADD_DEFINITIONS("-DCPPAD_PROFILE_SPEED ${cppad_profile_flag}")

# Extra flags used by the linker when creating an executable.
SET(CMAKE_EXE_LINKER_FLAGS ${cppad_profile_flag})

# Local include directories to search (not in package_prefix/includdir)
INCLUDE_DIRECTORIES( ${CMAKE_CURRENT_SOURCE_DIR}/../src )

# add_executable(<name> [WIN32] [MACOSX_BUNDLE] [EXCLUDE_FROM_ALL]
#                 source1 source2 ... sourceN
# )
SET(source_list ../main.cpp
    ../cppad/det_lu.cpp
    ../cppad/det_minor.cpp
    ../cppad/mat_mul.cpp
    ../cppad/ode.cpp
    ../cppad/poly.cpp
    ../cppad/sparse_hessian.cpp
    ../cppad/sparse_jacobian.cpp
)
set_compile_flags( speed_profile "${cppad_debug_which}" "${source_list}" )
#
ADD_EXECUTABLE( speed_profile EXCLUDE_FROM_ALL ${source_list} )

# profile builds it own copy of src/speed library (see ADD_EXECUTABLE above)
# TARGET_LINK_LIBRARIES(speed_profile speed_src )
TARGET_LINK_LIBRARIES(speed_profile
    speed_src
    ${cppad_lib}
    ${colpack_libs}
)

# Add the check_speed_profile target
ADD_CUSTOM_TARGET(check_speed_profile
    speed_profile correct 54321
    DEPENDS speed_profile
)
MESSAGE(STATUS "make check_speed_profile: available")

# Sed script to make gprof output more readable, use:
#   ./speed_profile speed 54321
#   gprof speed_profile gmon.out | sed -f 'gprof.sed' > gprof.out
#
# configure_file(InputFile OutputFile [COPYONLY] [ESCAPE_QUOTES] [@ONLY])
CONFIGURE_FILE(
    ${CMAKE_CURRENT_SOURCE_DIR}/gprof.sed.in
    ${CMAKE_CURRENT_BINARY_DIR}/gprof.sed
    COPYONLY
)

# Change check depends in parent environment
add_to_list(check_speed_depends check_speed_profile)
SET(check_speed_depends "${check_speed_depends}" PARENT_SCOPE)
