# ----------------------------------------------------------------------------
#  CppADCodeGen: C++ Algorithmic Differentiation with Source Code Generation:
#    Copyright (C) 2013 Ciengis
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
# - Try to find Clang
# Once done this will define
# CLANG_FOUND        - True if Clang found.
# CLANG_INCLUDE_DIRS - where to find Clang include files
# CLANG_LIBS         - list of clang libs

UNSET(CLANG_INCLUDE_DIRS CACHE)
UNSET(CLANG_LIBS CACHE)

IF(NOT LLVM_INCLUDE_DIRS OR NOT LLVM_LIBRARY_DIRS)
  SET(CLANG_FOUND FALSE)
  MESSAGE(WARNING "No LLVM and Clang support requires LLVM")
  RETURN()
ENDIF()

MACRO(findClangStaticLib _libname_)
  UNSET(CLANG_${_libname_}_LIB CACHE)
  IF(LLVM_LIBRARY_DIRS)
      FIND_LIBRARY(CLANG_${_libname_}_LIB ${_libname_} PATH ${LLVM_LIBRARY_DIRS})
  ELSE()
      FIND_LIBRARY(CLANG_${_libname_}_LIB ${_libname_} ${LLVM_LIBRARY_DIRS} ${CLANG_LIBRARY_DIRS})
  ENDIF()


  MARK_AS_ADVANCED(CLANG_${_libname_}_LIB)
  IF(CLANG_${_libname_}_LIB)
     SET(CLANG_LIBS ${CLANG_LIBS} ${CLANG_${_libname_}_LIB})
  ENDIF()
ENDMACRO()

findClangStaticLib(clang NAMES clang libclang) # LibClang: high-level C interface

# Clang shared library provides just the limited C interface, so it
# can not be used.  We look for the static libraries.
SET(CLANG_LIBNAMES clangCodeGen clangFrontendTool clangFrontend clangDriver clangSerialization clangTooling
                   clangParse clangSema clangChecker clangRewrite clangRewriteFrontend
                   clangStaticAnalyzerFrontend clangStaticAnalyzerCheckers
                   clangStaticAnalyzerCore clangAnalysis clangARCMigrate clangEdit clangAST clangASTMatchers clangLex
                   clangBasic clangRewriteCore)

foreach(LIBNAME ${CLANG_LIBNAMES})
    findClangStaticLib(${LIBNAME})
endforeach()

UNSET(CLANG_INCLUDE_DIRS CACHE)
FIND_PATH(CLANG_INCLUDE_DIRS clang/Basic/Version.h HINTS ${LLVM_INCLUDE_DIRS})

INCLUDE(FindPackageHandleStandardArgs)
# handle the QUIETLY and REQUIRED arguments and set CLANG_FOUND to TRUE
# if all listed variables are TRUE
find_package_handle_standard_args(CLANG  DEFAULT_MSG
                                  CLANG_INCLUDE_DIRS
                                  CLANG_LIBS)

MARK_AS_ADVANCED(CLANG_INCLUDE_DIRS CLANG_LIBS)
