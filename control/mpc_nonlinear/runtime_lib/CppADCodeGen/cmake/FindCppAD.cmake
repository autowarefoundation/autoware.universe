# ----------------------------------------------------------------------------
#  CppADCodeGen: C++ Algorithmic Differentiation with Source Code Generation:
#    Copyright (C) 2012 Ciengis
#    Copyright (C) 2020 Joao Leal
#
#  CppADCodeGen is distributed under multiple licenses:
#
#   - Eclipse Public License Version 1.0 (EPL1), and
#   - GNU General Public License Version 3 (GPL3).
#
#  EPL1 terms and conditions can be found in the file "epl-v10.txt", while
#  terms and conditions for the GPL3 can be found in the file "gpl3.txt".
# ----------------------------------------------------------------------------
#
# - Try to find CppAD
# Once done this will define
#  CPPAD_FOUND - System has CppAD
#  CPPAD_INCLUDE_DIRS - The CppAD include directories
#  CPPAD_LIBRARY_DIRS - The library directories needed to use CppAD
#  CPPAD_LIBRARIES    - The libraries needed to use CppAD

IF (CPPAD_INCLUDES AND CPPAD_LIBRARIES)
    SET(CPPAD_FIND_QUIETLY TRUE)
ENDIF ()


IF (DEFINED CPPAD_HOME)

    FIND_PATH(CPPAD_INCLUDE_DIR NAMES cppad/cppad.hpp
            PATHS "${CPPAD_HOME}"
            NO_DEFAULT_PATH)

    FIND_LIBRARY(CPPAD_IPOPT_LIBRARY
            cppad_ipopt
            PATHS "${CPPAD_HOME}/lib"
            NO_DEFAULT_PATH)

    SET(CPPAD_INCLUDE_DIRS ${CPPAD_INCLUDE_DIR})
    SET(CPPAD_LIBRARIES ${CPPAD_IPOPT_LIBRARY})
    SET(CPPAD_FOUND TRUE)

ELSE ()

    FIND_PACKAGE(PkgConfig)

    IF (PKG_CONFIG_FOUND)
        pkg_check_modules(CPPAD QUIET cppad)
    ENDIF ()


    IF (NOT CPPAD_FOUND)

        FIND_PATH(CPPAD_INCLUDE_DIR NAMES cppad/cppad.hpp
                HINTS "$ENV{CPPAD_HOME}"
                "/usr/include")

        FIND_LIBRARY(CPPAD_IPOPT_LIBRARY
                cppad_ipopt
                HINTS "$ENV{CPPAD_HOME}/lib"
                "/usr/lib")

        IF (CPPAD_INCLUDE_DIR)
            SET(CPPAD_INCLUDE_DIRS ${CPPAD_INCLUDE_DIR})
        ENDIF ()

        IF (CPPAD_IPOPT_LIBRARY)
            SET(CPPAD_LIBRARIES ${CPPAD_IPOPT_LIBRARY})
        ENDIF ()

        INCLUDE(FindPackageHandleStandardArgs)
        # handle the QUIETLY and REQUIRED arguments and set CPPAD_FOUND to TRUE
        # if all listed variables are TRUE
        find_package_handle_standard_args(CppAD DEFAULT_MSG
                CPPAD_INCLUDE_DIRS)

        MARK_AS_ADVANCED(CPPAD_INCLUDE_DIRS CPPAD_LIBRARIES)

    ENDIF ()
ENDIF ()

IF (CPPAD_FOUND)

    IF (CppAD_FIND_VERSION)

        IF (NOT CPPAD_VERSION)
            FILE(STRINGS ${CPPAD_INCLUDE_DIR}/cppad/configure.hpp CPPAD_VERSION_LINE
                    REGEX "CPPAD_PACKAGE_STRING +\"cppad-[0-9]+\\.[0-9]+\"")
            STRING(REGEX MATCH "[0-9]+\\.[0-9]+" CPPAD_VERSION "${CPPAD_VERSION_LINE}")
        ENDIF ()

        IF (CppAD_FIND_VERSION_EXACT)
            IF (NOT "${CPPAD_VERSION}" VERSION_EQUAL "${CppAD_FIND_VERSION}")
                SET(CPPAD_FOUND FALSE)
                MESSAGE(FATAL_ERROR "Found CppAD version '${CPPAD_VERSION}' but version '${CppAD_FIND_VERSION}' is required")
            ENDIF ()
        ELSE ()
            IF ("${CPPAD_VERSION}" VERSION_LESS "${CppAD_FIND_VERSION}")
                SET(CPPAD_FOUND FALSE)
                MESSAGE(FATAL_ERROR "Found CppAD version '${CPPAD_VERSION}' but at least version '${CppAD_FIND_VERSION}' is required")
            ENDIF ()
        ENDIF ()

    ENDIF ()

    IF (CPPAD_FOUND AND NOT CPPAD_FIND_QUIETLY)
        MESSAGE(STATUS "package CppAD ${CPPAD_VERSION} found")
    ENDIF ()

ENDIF ()

