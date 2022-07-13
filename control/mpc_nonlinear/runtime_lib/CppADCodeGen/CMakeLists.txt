# --------------------------------------------------------------------------
#  CppADCodeGen: C++ Algorithmic Differentiation with Source Code Generation:
#    Copyright (C) 2018 Joao Leal
#    Copyright (C) 2012 Ciengis
#
#  CppADCodeGen is distributed under multiple licenses:
#
#   - Eclipse Public License Version 1.0 (EPL1), and
#   - GNU General Public License Version 3 (GPL3).
#
# EPL1 terms and conditions can be found in the file "epl-v10.txt", while
# terms and conditions for the GPL3 can be found in the file "gpl3.txt".
# ----------------------------------------------------------------------------
#
# Author: Joao Leal
#
# ----------------------------------------------------------------------------
CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

PROJECT(cppadcg CXX C)

SET(cppadcg_version      "2.4.3" )
SET(cppadcg_url          "https://github.com/joaoleal/CppADCodeGen" )
SET(cppadcg_description  "A C++ Algorithmic Differentiation Package with Source Code Generation" )

# ----------------------------------------------------------------------------
#                                  options
# ----------------------------------------------------------------------------
set(LLVM_VERSION "" CACHE STRING "Prefered LLVM and Clang version to be used in tests")

# ----------------------------------------------------------------------------
#                      some additional cmake modules
# ----------------------------------------------------------------------------
SET(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

# ----------------------------------------------------------------------------
#                          find dependencies
# ----------------------------------------------------------------------------
FIND_PACKAGE(Threads REQUIRED)

SET(cppad_version_min 20200000.1)
FIND_PACKAGE(CppAD ${cppad_version_min} REQUIRED)

FIND_PACKAGE(ADOLC)

FIND_PACKAGE(Eigen3)

IF(UNIX)
    FIND_PACKAGE(DL REQUIRED)
ENDIF()

FIND_PACKAGE(LLVM ${LLVM_VERSION})
FIND_PACKAGE(Clang ${LLVM_VERSION})

IF(LLVM_FOUND)
    ADD_DEFINITIONS(-DLLVM_VERSION_MAJOR=${LLVM_VERSION_MAJOR}
                    -DLLVM_VERSION_MINOR=${LLVM_VERSION_MINOR})
ENDIF()

FIND_PACKAGE(OpenMP)

# Latex used for test and example
FIND_PACKAGE(LATEX)

# graphviz used for test
FIND_PROGRAM(GRAPHVIZ_DOT_PATH dot)


#
#             Doxygen
#
OPTION(CREATE_DOXYGEN_DOC "Build documentation" OFF)
IF(${CREATE_DOXYGEN_DOC})
    FIND_PACKAGE(Doxygen REQUIRED)
ELSE()
    FIND_PACKAGE(Doxygen)
ENDIF()

###
# cppcheck tests
###
INCLUDE(cmake/cppcheck.cmake)

###
# some utilities
###
INCLUDE(cmake/util.cmake)
INCLUDE(cmake/textfile2h.cmake)

# ----------------------------------------------------------------------------
#                                 Options
# ----------------------------------------------------------------------------
OPTION(ENABLE_TEST_CPPCHECKS "Use CppCheck to verify source code in tests (very lengthy task!)" OFF)
OPTION(ENABLE_THREAD_POOL_TESTS "Enables tests for thread pool" ON)

# ----------------------------------------------------------------------------
#                                 Includes
# ----------------------------------------------------------------------------
INCLUDE_DIRECTORIES(${CPPAD_INCLUDE_DIRS} "${CMAKE_SOURCE_DIR}/include" "${CMAKE_BINARY_DIR}/include" ${DL_INCLUDE_DIRS})

IF(EIGEN3_FOUND)
    INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIR})
ENDIF()

# ----------------------------------------------------------------------------
#                     General installation information
# ----------------------------------------------------------------------------
IF(UNIX)
    SET(CMAKE_INSTALL_PREFIX "/usr" CACHE PATH "Default install path prefix")
ENDIF()

SET(install_cppadcg_include_location     "${CMAKE_INSTALL_PREFIX}/include/cppad")
SET(install_cppadcg_include_pkg_location "${CMAKE_INSTALL_PREFIX}/include")
SET(install_cppad_include_location       "${CMAKE_INSTALL_PREFIX}/include/cppad")
SET(install_cppad_include_pkg_location   "${CMAKE_INSTALL_PREFIX}/include")
SET(install_library_pkg_location         "${CMAKE_INSTALL_PREFIX}/share/pkgconfig")
SET(install_doc_location                 "${CMAKE_INSTALL_PREFIX}/share/doc/cppadcg")
SET(install_python_location              "${CMAKE_INSTALL_PREFIX}/share/cppadcg/python")

# ----------------------------------------------------------------------------
#                    Define some optional compiler flags
# ----------------------------------------------------------------------------
IF(CMAKE_VERSION VERSION_GREATER "3.1.0")
    # aim for this version which has explicit support for c++11
    SET(CMAKE_CXX_STANDARD 11)
    SET(CMAKE_CXX_STANDARD_REQUIRED TRUE)
ENDIF()

IF(CMAKE_COMPILER_IS_GNUCXX OR "${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
    SET(warning_flags "-Wall") # enable as many warnings as possible
    #SET(warning_flags "${warning_flags} -Wextra")
    #SET(warning_flags "${warning_flags} -Wshadow") # definition shadows another
    SET(warning_flags "${warning_flags} -Wno-system-headers") # do not issue warnings for system headers
    #SET(warning_flags "${warning_flags} -Woverloaded-virtual") # overloaded virtual function has a different signature

    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${warning_flags}")

    IF(CMAKE_VERSION VERSION_GREATER "3.1.0")
        # do nothing
    ELSE()
        SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
    ENDIF()


    SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O0 -g -ggdb")

    # NDEBUG required in order to make CppAD significantly faster
    SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O2 -DNDEBUG")

    SET(CMAKE_CXX_FLAGS_RELWITHDEBINFO "-O2 -g -ggdb -DNDEBUG -D_FORTIFY_SOURCE=2")
ENDIF()


# ----------------------------------------------------------------------------
# Check if it is possible to use LLVM/Clang for JIT
# ----------------------------------------------------------------------------
SET(CPPADCG_LLVM_LINK_LIB "3.2|3.6|3.8|4.0|5.0|6.0|7.0|8.0|9.0")

IF((("${LLVM_VERSION_MAJOR}.${LLVM_VERSION_MINOR}" MATCHES "^(${CPPADCG_LLVM_LINK_LIB})$") AND CLANG_FOUND)
        OR
    ("${LLVM_VERSION_MAJOR}.${LLVM_VERSION_MINOR}" MATCHES "^(3.3|3.4)$")
  )
    SET(CPPADCG_USE_LLVM ON)
ELSE()
    SET(CPPADCG_USE_LLVM OFF)
ENDIF()

# ----------------------------------------------------------------------------
# Currently building tests as normal executables
# ----------------------------------------------------------------------------
ADD_SUBDIRECTORY(include)
ADD_SUBDIRECTORY(example EXCLUDE_FROM_ALL)
#ADD_SUBDIRECTORY(introduction EXCLUDE_FROM_ALL)
ADD_SUBDIRECTORY(speed EXCLUDE_FROM_ALL)
ADD_SUBDIRECTORY(python)
ADD_SUBDIRECTORY(pkgconfig)
ADD_SUBDIRECTORY(doc)

# enable testing must be called here in order for the target test to be created
ENABLE_TESTING()
ADD_SUBDIRECTORY(test EXCLUDE_FROM_ALL)

# =============================================================================
# uninstall procedure
# =============================================================================
ADD_CUSTOM_TARGET(uninstall COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_SOURCE_DIR}/cmake/cppadcodegen_uninstall.cmake)
