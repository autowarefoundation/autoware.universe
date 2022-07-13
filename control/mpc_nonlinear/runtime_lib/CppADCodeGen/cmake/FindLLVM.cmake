# ----------------------------------------------------------------------------
#  CppADCodeGen: C++ Algorithmic Differentiation with Source Code Generation:
#    Copyright (C) 2013 Ciengis
#    Copyright (C) 2019 Joao Leal
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
# - Try to find LLVM
#
# It defines the following variables
#  LLVM_FOUND            - True if llvm found.
#  LLVM_INCLUDE_DIRS     - where to find llvm include files
#  LLVM_LIBRARY_DIRS     - where to find llvm libs
#  LLVM_CFLAGS           - llvm compiler flags
#  LLVM_CFLAGS_NO_NDEBUG - llvm compiler flags without NDEBUG
#  LLVM_WITH_NDEBUG      - 1 if LLVM_CFLAGS contains NDEBUG, 0 otherwise
#  LLVM_LDFLAGS          - llvm linker flags
#  LLVM_MODULE_LIBS      - list of llvm libs for working with modules.

IF(LLVM_FIND_VERSION AND LLVM_FIND_VERSION_MAJOR LESS 4 AND NOT LLVM_FIND_VERSION_MINOR)
    MESSAGE(FATAL_ERROR "When requesting a specific version of LLVM higher or equal to 7.0 you should only proide the major version, on LLVM versions lower than 4.0, you must provide at least the major and minor version numbers, e.g., 3.8")
ENDIF()

# Lets make sure cache doesn't ruin the day
UNSET(LLVM_CONFIG CACHE)
UNSET(LLVM_FOUND CACHE)
UNSET(LLVM_FOUND CACHE)
UNSET(LLVM_INCLUDE_DIRS CACHE)
UNSET(LLVM_LIBRARY_DIRS CACHE)
UNSET(LLVM_CFLAGS CACHE)
UNSET(LLVM_LDFLAGS CACHE)
UNSET(LLVM_MODULE_LIBS CACHE)


MACRO(find_llvm_iteratively)
    IF(NOT LLVM_CONFIG AND NOT LLVM_FIND_VERSION_EXACT)
        SET(_LLVM_KNOWN_VERSIONS ${LLVM_ADDITIONAL_VERSIONS} "9" "8" "7" "6.0" "5.0" "4.0" "3.8" "3.7" "3.6" "3.5" "3.4" "3.3" "3.2")

        # Select acceptable versions.
        FOREACH(version ${_LLVM_KNOWN_VERSIONS})

            IF(NOT "${version}" VERSION_LESS "${LLVM_FIND_VERSION_MAJOR}.${LLVM_FIND_VERSION_MINOR}")
                FIND_PROGRAM(LLVM_CONFIG "llvm-config-${version}")
                IF(LLVM_CONFIG)
                    BREAK() # Found suitable version
                ENDIF()
            ELSE()
                BREAK() # Lower version than requested
            ENDIF()
        ENDFOREACH()
    ENDIF()
ENDMACRO()


IF(LLVM_FIND_VERSION)
    FIND_PROGRAM(LLVM_CONFIG "llvm-config-${LLVM_VERSION}")

    IF(NOT LLVM_CONFIG) # LLVM_FIND_VERSION_EXACT or failed to find using previous search
        # Lets try to find the exact specified version
        FIND_PROGRAM(LLVM_CONFIG "llvm-config-${LLVM_FIND_VERSION_MAJOR}.${LLVM_FIND_VERSION_MINOR}")

        IF(NOT LLVM_CONFIG AND NOT LLVM_FIND_VERSION_EXACT)
            find_llvm_iteratively()
        ENDIF()
    ENDIF()
ELSE()
    FIND_PROGRAM(LLVM_CONFIG llvm-config)

    IF(NOT LLVM_CONFIG)
        find_llvm_iteratively()
    ENDIF()
ENDIF()


IF(LLVM_CONFIG)
    MESSAGE(STATUS "llvm-config found at: ${LLVM_CONFIG}")
ELSE()
    MESSAGE(WARNING "Could NOT find llvm-config")
    SET(LLVM_FOUND FALSE)
    return()
ENDIF()

EXECUTE_PROCESS(COMMAND ${LLVM_CONFIG} --version
                OUTPUT_VARIABLE LLVM_VERSION
                OUTPUT_STRIP_TRAILING_WHITESPACE)

STRING(REGEX REPLACE "^([0-9]+)\\.([0-9]+).*" "\\1"
       LLVM_VERSION_MAJOR
       "${LLVM_VERSION}")

STRING(REGEX REPLACE "^([0-9]+)\\.([0-9]+).*" "\\2"
       LLVM_VERSION_MINOR
       "${LLVM_VERSION}")

# Version validation
IF(LLVM_FIND_VERSION)
  IF(LLVM_FIND_VERSION_EXACT)
    IF(NOT ${LLVM_VERSION} VERSION_EQUAL "${LLVM_FIND_VERSION_MAJOR}.${LLVM_FIND_VERSION_MINOR}")
      MESSAGE(FATAL_ERROR "Failed to find the specified version of LLVM")
    ENDIF()
  ELSEIF(${LLVM_VERSION} VERSION_LESS "${LLVM_FIND_VERSION_MAJOR}.${LLVM_FIND_VERSION_MINOR}")
    MESSAGE(FATAL_ERROR "Failed to find a LLVM version equal or higher than ${LLVM_FIND_VERSION_MAJOR}.${LLVM_FIND_VERSION_MINOR}")
  ENDIF()
ENDIF()


EXECUTE_PROCESS(COMMAND ${LLVM_CONFIG} --includedir
                OUTPUT_VARIABLE LLVM_INCLUDE_DIRS
                OUTPUT_STRIP_TRAILING_WHITESPACE)

EXECUTE_PROCESS(COMMAND ${LLVM_CONFIG} --libdir
                OUTPUT_VARIABLE LLVM_LIBRARY_DIRS
                OUTPUT_STRIP_TRAILING_WHITESPACE)

EXECUTE_PROCESS(COMMAND ${LLVM_CONFIG} --cppflags
                OUTPUT_VARIABLE LLVM_CFLAGS
                OUTPUT_STRIP_TRAILING_WHITESPACE)

IF(LLVM_CFLAGS MATCHES "\\-DNDEBUG")
    SET(LLVM_WITH_NDEBUG 1)
ELSE()
    SET(LLVM_WITH_NDEBUG 0)
ENDIF()

STRING(REPLACE "-DNDEBUG" ""
       LLVM_CFLAGS_NO_NDEBUG
       ${LLVM_CFLAGS})

FIND_LIBRARY(LLVM_MODULE_LIBS LLVM-${LLVM_VERSION_MAJOR}.${LLVM_VERSION_MINOR} ${LLVM_LIBRARY_DIRS})
IF(NOT LLVM_MODULE_LIBS)
  EXECUTE_PROCESS(COMMAND ${LLVM_CONFIG} --libs
                  OUTPUT_VARIABLE LLVM_MODULE_LIBS
                  OUTPUT_STRIP_TRAILING_WHITESPACE)
ENDIF()

EXECUTE_PROCESS(COMMAND ${LLVM_CONFIG} --ldflags
                OUTPUT_VARIABLE LLVM_LDFLAGS
                OUTPUT_STRIP_TRAILING_WHITESPACE)

INCLUDE(FindPackageHandleStandardArgs)
# handle the QUIETLY and REQUIRED arguments and set LLVM_FOUND to TRUE
# if all listed variables are TRUE
find_package_handle_standard_args(LLVM  DEFAULT_MSG
                                  LLVM_INCLUDE_DIRS
                                  LLVM_LIBRARY_DIRS
                                  LLVM_CFLAGS
                                  LLVM_LDFLAGS
                                  LLVM_MODULE_LIBS)

MARK_AS_ADVANCED(LLVM_INCLUDE_DIRS
                 LLVM_LIBRARY_DIRS
                 LLVM_CFLAGS
                 LLVM_LDFLAGS
                 LLVM_MODULE_LIBS)
