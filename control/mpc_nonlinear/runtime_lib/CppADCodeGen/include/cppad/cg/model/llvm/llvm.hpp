#ifndef CPPAD_CG_LLVM_INCLUDED
#define CPPAD_CG_LLVM_INCLUDED
/* --------------------------------------------------------------------------
 *  CppADCodeGen: C++ Algorithmic Differentiation with Source Code Generation:
 *    Copyright (C) 2014 Ciengis
 *    Copyright (C) 2019 Joao Leal
 *
 *  CppADCodeGen is distributed under multiple licenses:
 *
 *   - Eclipse Public License Version 1.0 (EPL1), and
 *   - GNU General Public License Version 3 (GPL3).
 *
 *  EPL1 terms and conditions can be found in the file "epl-v10.txt", while
 *  terms and conditions for the GPL3 can be found in the file "gpl3.txt".
 * ----------------------------------------------------------------------------
 * Author: Joao Leal
 */

#ifdef LLVM_VERSION_MAJOR

#if LLVM_VERSION_MAJOR==3 && LLVM_VERSION_MINOR==2
#include <cppad/cg/model/llvm/v3_2/llvm3_2.hpp>
#elif LLVM_VERSION_MAJOR==3 && (LLVM_VERSION_MINOR==3 || LLVM_VERSION_MINOR==4)
#include <cppad/cg/model/llvm/v3_4/llvm3_4.hpp>
#elif LLVM_VERSION_MAJOR==3 && LLVM_VERSION_MINOR==6
#include <cppad/cg/model/llvm/v3_6/llvm3_6.hpp>
#elif LLVM_VERSION_MAJOR==3 && LLVM_VERSION_MINOR==8
#include <cppad/cg/model/llvm/v3_8/llvm3_8.hpp>
#elif LLVM_VERSION_MAJOR==4 && LLVM_VERSION_MINOR==0
#include <cppad/cg/model/llvm/v4_0/llvm4_0.hpp>
#elif LLVM_VERSION_MAJOR==5 && LLVM_VERSION_MINOR==0
#include <cppad/cg/model/llvm/v5_0/llvm5_0.hpp>
#elif LLVM_VERSION_MAJOR==6 && LLVM_VERSION_MINOR==0
#include <cppad/cg/model/llvm/v6_0/llvm6_0.hpp>
#elif LLVM_VERSION_MAJOR==7 && LLVM_VERSION_MINOR==0
#include <cppad/cg/model/llvm/v7_0/llvm7_0.hpp>
#elif LLVM_VERSION_MAJOR==8 && LLVM_VERSION_MINOR==0
#include <cppad/cg/model/llvm/v8_0/llvm8_0.hpp>
#elif LLVM_VERSION_MAJOR==9 && LLVM_VERSION_MINOR==0
#include <cppad/cg/model/llvm/v9_0/llvm9_0.hpp>
#endif

#endif

#endif
