# ----------------------------------------------------------------------------
#  CppADCodeGen: C++ Algorithmic Differentiation with Source Code Generation:
#    Copyright (C) 2012 Ciengis
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
# - Try to find libdl
# Once done this will define
#  DL_FOUND - System has libdl
#  DL_INCLUDE_DIRS - The libdl include directories
#  DL_LIBRARY_DIRS - The library directories needed to use libdl
#  DL_LIBRARIES    - The libraries needed to use libdl

if (DL_INCLUDES AND DL_LIBRARIES)
  set(DL_FIND_QUIETLY TRUE)
endif (DL_INCLUDES AND DL_LIBRARIES)


FIND_PACKAGE(PkgConfig)
IF( PKG_CONFIG_FOUND )

  pkg_check_modules( DL QUIET dl)

ENDIF( PKG_CONFIG_FOUND )


IF( DL_FOUND )
  IF(NOT DL_FIND_QUIETLY)
    MESSAGE(STATUS "package dl found")
  ENDIF()
ELSE( DL_FOUND )
  FIND_PATH(DL_INCLUDE_DIRS NAMES dlfcn.h
            HINTS  $ENV{DL_HOME}
                   "/usr/include/" )
           
  FIND_LIBRARY(DL_LIBRARY 
               dl ltdl
                HINTS "$ENV{DL_HOME}/lib"
                      "/usr/lib" )

  SET(DL_INCLUDE_DIRS ${DL_INCLUDE_DIR})
  SET(DL_LIBRARIES ${DL_LIBRARY})

  INCLUDE(FindPackageHandleStandardArgs)
  # handle the QUIETLY and REQUIRED arguments and set LIBDL_FOUND to TRUE
  # if all listed variables are TRUE
  find_package_handle_standard_args(DL  DEFAULT_MSG
                                    DL_INCLUDE_DIRS
                                    DL_LIBRARIES)

  MARK_AS_ADVANCED(DL_INCLUDE_DIRS DL_LIBRARIES)
  
  IF( DL_FOUND AND NOT DL_FIND_QUIETLY )
    MESSAGE(STATUS "package dl found")
  ENDIF()
ENDIF( DL_FOUND )
