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
# Build the speed/adolc directory tests
# Inherit build type from ../CMakeList.txt

# assert include_adolc is true
assert(include_adolc )


# Adds flags to the compiler command line for sources in the current directory
# and below. This command can be used to add any flags, but it was originally
# intended to add preprocessor definitions.
ADD_DEFINITIONS("-DCPPAD_ADOLC_SPEED")

# Local include directories to search (not in package_prefix/includdir)
INCLUDE_DIRECTORIES( ${CMAKE_CURRENT_SOURCE_DIR}/../src )

# add_executable(<name> [WIN32] [MACOSX_BUNDLE] [EXCLUDE_FROM_ALL]
#                 source1 source2 ... sourceN
# )
SET(source_list ../main.cpp alloc_mat.cpp
    det_lu.cpp
    det_minor.cpp
    mat_mul.cpp
    ode.cpp
    poly.cpp
    sparse_hessian.cpp
    sparse_jacobian.cpp
)
set_compile_flags( speed_adolc "${cppad_debug_which}" "${source_list}" )
#
ADD_EXECUTABLE( speed_adolc EXCLUDE_FROM_ALL ${source_list} )

# List of libraries to be linked into the specified target
TARGET_LINK_LIBRARIES(speed_adolc
    speed_src
    ${cppad_lib}
    adolc
    ${colpack_libs}
)

# Add the check_speed_adolc target
ADD_CUSTOM_TARGET(
    check_speed_adolc
    speed_adolc correct 54321 colpack
    DEPENDS
    speed_adolc speed_src
)
MESSAGE(STATUS "make check_speed_adolc: available")

# Change check depends in parent environment
add_to_list(check_speed_depends check_speed_adolc)
SET(check_speed_depends "${check_speed_depends}" PARENT_SCOPE)
