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

# initialize list as empty
SET(check_example_multi_thread_depends "")

# Define the operation
# CHECK_LIBRARY_EXISTS (LIBRARY FUNCTION LOCATION VARIABLE)
#   LIBRARY  - the name of the library you are looking for
#   FUNCTION - the name of the function
#   LOCATION - location where the library should be found
#   VARIABLE - variable to store the result
INCLUDE(CheckLibraryExists)

# If openmp found, set process its subdirectory
IF ( OPENMP_FOUND )
    # OpenMP_CXX_FLAGS - flags to add to the CXX compiler for OpenMP support
    ADD_SUBDIRECTORY(openmp)
ENDIF ( OPENMP_FOUND )

# If pthreads found, set pthread_lib and pthread_lib_path and process subdir
#
# find_library(<VAR> name1 [path1 path2 ...])
SET(pthread_lib "pthread")
FIND_LIBRARY(pthread_lib_path pthread)
MESSAGE(STATUS "pthread library path = ${pthread_lib_path}")
IF ( pthread_lib_path )
    CHECK_LIBRARY_EXISTS(
        pthread pthread_barrier_wait ${pthread_lib_path} pthread_ok
    )
    IF ( pthread_ok )
        ADD_SUBDIRECTORY(pthread)
    ENDIF ( pthread_ok )
ENDIF ( pthread_lib_path )

# Check if boost multi-threading library is avaialble
IF ( Boost_FOUND )
    SET(CMAKE_REQUIRED_DEFINITIONS "")
    SET(CMAKE_REQUIRED_FLAGS       "")
    SET(CMAKE_REQUIRED_INCLUDES    "${Boost_INCLUDE_DIRS}")
    SET(CMAKE_REQUIRED_LIBRARIES   "${Boost_LIBRARIES}")
    SET(source "
# include <boost/thread.hpp>
int main(void)
{   boost::barrier wait(1);
    return 0;
}"
    )
    compile_source_test("${source}" boost_multi_thread_ok )
    #
    IF( boost_multi_thread_ok )
        ADD_SUBDIRECTORY(bthread)
    ENDIF( boost_multi_thread_ok )
    #
ENDIF ( Boost_FOUND )

IF( NOT( "${check_example_multi_thread_depends}" STREQUAL "" ) )
    #
    # check_example_multi_thread
    ADD_CUSTOM_TARGET(
        check_example_multi_thread
        DEPENDS ${check_example_multi_thread_depends}
    )
    MESSAGE(STATUS "make check_example_multi_thread: available")
    #
    # Change check depends in parent environment
    add_to_list(check_example_depends check_example_multi_thread)
    SET(check_example_depends "${check_example_depends}" PARENT_SCOPE)
    #
ENDIF( NOT( "${check_example_multi_thread_depends}" STREQUAL "" ) )
