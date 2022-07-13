#ifndef CPPAD_CG_LLVM_MODEL_LIBRARY_3_2_INCLUDED
#define CPPAD_CG_LLVM_MODEL_LIBRARY_3_2_INCLUDED
/* --------------------------------------------------------------------------
 *  CppADCodeGen: C++ Algorithmic Differentiation with Source Code Generation:
 *    Copyright (C) 2013 Ciengis
 *    Copyright (C) 2018 Joao Leal
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

namespace CppAD {
namespace cg {

template<class Base> class LlvmModel;

/**
 * Class used to load JIT'ed models by LLVM 3.2
 *
 * @author Joao Leal
 */
template<class Base>
class LlvmModelLibrary3_2 : public LlvmModelLibrary<Base> {
protected:
    llvm::Module* _module;
    std::unique_ptr<llvm::LLVMContext> _context;
    std::unique_ptr<llvm::ExecutionEngine> _executionEngine;
    std::unique_ptr<llvm::FunctionPassManager> _fpm;
public:

    LlvmModelLibrary3_2(llvm::Module* module,
                        llvm::LLVMContext* context) :
        _module(module),
        _context(context) {
        using namespace llvm;

        // Create the JIT.  This takes ownership of the module.
        std::string errStr;
        _executionEngine.reset(EngineBuilder(_module)
                               .setErrorStr(&errStr)
                               .setEngineKind(EngineKind::JIT)
                               .create());
        if (!_executionEngine.get()) {
            throw CGException("Could not create ExecutionEngine: ", errStr);
        }

        _fpm.reset(new llvm::FunctionPassManager(_module));

        preparePassManager();

        _fpm->doInitialization();

        /**
         *
         */
        this->validate();
    }

    LlvmModelLibrary3_2(const LlvmModelLibrary3_2&) = delete;
    LlvmModelLibrary3_2& operator=(const LlvmModelLibrary3_2&) = delete;

    inline virtual ~LlvmModelLibrary3_2() {
        this->cleanUp();
    }

    /**
     * Set up the optimizer pipeline
     */
    virtual void preparePassManager() {
        llvm::PassManagerBuilder builder;
        builder.OptLevel = 2;
        builder.populateFunctionPassManager(*_fpm);

        /*
        // Set up the optimizer pipeline.  Start with registering info about how the
        // target lays out data structures.
        _fpm->add(new DataLayout(*_executionEngine->getDataLayout()));
        // Provide basic AliasAnalysis support for GVN.
        _fpm->add(createBasicAliasAnalysisPass());
        // Do simple "peephole" optimizations and bit-twiddling optzns.
        _fpm->add(createInstructionCombiningPass());
        // Re-associate expressions.
        _fpm->add(createReassociatePass());
        // Eliminate Common SubExpressions.
        _fpm->add(createGVNPass());
        _fpm->add(llvm::createDeadStoreEliminationPass()); // Delete dead stores
        // Simplify the control flow graph (deleting unreachable blocks, etc).
        _fpm->add(createCFGSimplificationPass());
         */
    }

    void* loadFunction(const std::string& functionName, bool required = true) override {
        llvm::Function* func = _module->getFunction(functionName);
        if (func == nullptr) {
            if (required)
                throw CGException("Unable to find function '", functionName, "' in LLVM module");
            return nullptr;
        }

#ifndef NDEBUG
        // Validate the generated code, checking for consistency.
        llvm::verifyFunction(*func);
#endif
        // Optimize the function.
        _fpm->run(*func);

        // JIT the function, returning a function pointer.
        void *fPtr = _executionEngine->getPointerToFunction(func);
        return fPtr;
    }

    friend class LlvmModel<Base>;

};

} // END cg namespace
} // END CppAD namespace

#endif
