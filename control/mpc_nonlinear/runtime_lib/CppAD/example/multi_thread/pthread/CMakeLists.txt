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
# Build the example/multi_thread/pthread directory tests
# Inherit build type from ../CMakeList.txt

# Local include directories to search (not in package_prefix/include)
INCLUDE_DIRECTORIES( ${CMAKE_CURRENT_SOURCE_DIR}/.. )

# add_executable(<name> [WIN32] [MACOSX_BUNDLE] [EXCLUDE_FROM_ALL]
#                 source1 source2 ... sourceN
# )
SET(source_list ../thread_test.cpp
    ../team_example.cpp
    ../harmonic.cpp
    ../multi_atomic_two.cpp
    ../multi_atomic_three.cpp
    ../multi_chkpoint_one.cpp
    ../multi_chkpoint_two.cpp
    ../multi_newton.cpp
    a11c_pthread.cpp
    simple_ad_pthread.cpp
    team_pthread.cpp
)
set_compile_flags( example_multi_thread_pthread "${cppad_debug_which}" "${source_list}" )
#
ADD_EXECUTABLE( example_multi_thread_pthread EXCLUDE_FROM_ALL ${source_list} )

# Paths in which the linker will search for libraries,
# only applies to targets created after it is called
LINK_DIRECTORIES( ${pthread_lib_path} )

# List of libraries to be linked into the specified target
TARGET_LINK_LIBRARIES(example_multi_thread_pthread ${pthread_lib})

# Add the check_example_multi_thread_pthread target
ADD_CUSTOM_TARGET(check_example_multi_thread_pthread
    example_multi_thread_pthread simple_ad
    DEPENDS example_multi_thread_pthread
)
MESSAGE(STATUS "make check_example_multi_thread_pthread: available")

# Change check depends in parent environment
add_to_list(check_example_multi_thread_depends check_example_multi_thread_pthread)
SET(check_example_multi_thread_depends "${check_example_multi_thread_depends}" PARENT_SCOPE)
