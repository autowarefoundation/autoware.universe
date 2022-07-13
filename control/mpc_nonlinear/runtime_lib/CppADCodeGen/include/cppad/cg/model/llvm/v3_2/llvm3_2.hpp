#ifndef CPPAD_CG_LLVM3_2_INCLUDED
#define CPPAD_CG_LLVM3_2_INCLUDED
/* --------------------------------------------------------------------------
 *  CppADCodeGen: C++ Algorithmic Differentiation with Source Code Generation:
 *    Copyright (C) 2013 Ciengis
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

/**
 * LLVM requires the use of it own flags which can make it difficult to compile
 * libraries not using NDEBUG often required by LLVM.
 * The define LLVM_CPPFLAG_NDEBUG can be used to apply NDEBUG only to LLVM 
 * headers.
 */
#ifdef LLVM_WITH_NDEBUG

// save the original NDEBUG definition
#ifdef NDEBUG
#define _OUTER_NDEBUG_DEFINED
#endif

#if LLVM_WITH_NDEBUG == 1
#define NDEBUG
#else
#undef NDEBUG
#endif

#endif

#include <clang/CodeGen/CodeGenAction.h>
#include <clang/Basic/DiagnosticOptions.h>
#include <clang/Basic/TargetInfo.h>
#include <clang/Basic/SourceManager.h>
#include <clang/Frontend/CompilerInstance.h>
#include <clang/Frontend/CompilerInvocation.h>
#include <clang/Frontend/FrontendDiagnostic.h>
#include <clang/Frontend/TextDiagnosticPrinter.h>
#include <clang/Frontend/Utils.h>
#include <clang/Parse/ParseAST.h>
#include <clang/Lex/Preprocessor.h>

#include <llvm/Analysis/Passes.h>
#include <llvm/Analysis/Verifier.h>
#include <llvm/ExecutionEngine/ExecutionEngine.h>
#include <llvm/ExecutionEngine/JIT.h>
#include <llvm/PassManager.h>
#include <llvm/Module.h>
#include <llvm/Pass.h>
#include <llvm/Transforms/IPO/PassManagerBuilder.h>
#include <llvm/ADT/OwningPtr.h>
#include <llvm/ADT/SmallString.h>
#include <llvm/LLVMContext.h>
#include <llvm/Support/Host.h>
#include <llvm/Support/ManagedStatic.h>
#include <llvm/Support/Path.h>
#include <llvm/Support/TargetSelect.h>
#include <llvm/Support/raw_ostream.h>
#include <llvm/Support/MemoryBuffer.h>
#include <llvm/Support/Host.h>
#include <llvm/Linker.h>

#ifdef LLVM_WITH_NDEBUG

// recover the original NDEBUG
#ifdef _OUTER_NDEBUG_DEFINED
#define NDEBUG
#else
#undef NDEBUG
#endif

// no need for this anymore
#undef _OUTER_NDEBUG_DEFINED

#endif

#include <cppad/cg/model/llvm/llvm_model_library.hpp>
#include <cppad/cg/model/llvm/llvm_model.hpp>
#include <cppad/cg/model/llvm/v3_2/llvm_model_library_3_2.hpp>
#include <cppad/cg/model/llvm/v3_2/llvm_model_library_processor.hpp>

#endif