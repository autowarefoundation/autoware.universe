# --------------------------------------------------------------------------
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
# Author: Joao Leal
#
# ----------------------------------------------------------------------------
INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR} ${LLVM_INCLUDE_DIRS} ${CLANG_INCLUDE_DIRS} ${DL_INCLUDE_DIRS})

LINK_DIRECTORIES(${LLVM_LIBRARY_DIRS})
ADD_DEFINITIONS(${LLVM_CFLAGS_NO_NDEBUG} -DLLVM_WITH_NDEBUG=${LLVM_WITH_NDEBUG})

add_cppadcg_test(llvm_external_compiler.cpp)
add_cppadcg_test(llvm_link_clang.cpp)

TARGET_LINK_LIBRARIES(llvm_external_compiler
                      ${LLVM_MODULE_LIBS}
                      ${LLVM_LDFLAGS})

TARGET_LINK_LIBRARIES(llvm_link_clang
                      ${LLVM_MODULE_LIBS}
                      ${LLVM_LDFLAGS})

IF("${LLVM_VERSION_MAJOR}.${LLVM_VERSION_MINOR}" MATCHES "^(${CPPADCG_LLVM_LINK_LIB})$")
  TARGET_LINK_LIBRARIES(llvm_external_compiler
                        ${CLANG_LIBS})
  TARGET_LINK_LIBRARIES(llvm_link_clang
                        ${CLANG_LIBS})
ENDIF()
