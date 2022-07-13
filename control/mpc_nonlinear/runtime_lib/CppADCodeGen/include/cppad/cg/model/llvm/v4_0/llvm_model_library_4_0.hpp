#ifndef CPPAD_CG_LLVM_MODEL_LIBRARY_4_0_INCLUDED
#define CPPAD_CG_LLVM_MODEL_LIBRARY_4_0_INCLUDED
/* --------------------------------------------------------------------------
 *  CppADCodeGen: C++ Algorithmic Differentiation with Source Code Generation:
 *    Copyright (C) 2017 Ciengis
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
 * Class used to load JIT'ed models by LLVM 4.0
 *
 * @author Joao Leal
 */
template<class Base>
class LlvmModelLibrary4_0 : public LlvmModelLibrary<Base> {
protected:
    llvm::Module* _module; // owned by _executionEngine
    std::shared_ptr<llvm::LLVMContext> _context;
    std::unique_ptr<llvm::ExecutionEngine> _executionEngine;
    std::unique_ptr<llvm::legacy::FunctionPassManager> _fpm;
public:

    LlvmModelLibrary4_0(std::unique_ptr<llvm::Module> module,
                        std::shared_ptr<llvm::LLVMContext> context) :
        _module(module.get()),
        _context(context) {
        using namespace llvm;

        // Create the JIT.  This takes ownership of the module.
        std::string errStr;
        _executionEngine.reset(EngineBuilder(std::move(module))
                               .setErrorStr(&errStr)
                               .setEngineKind(EngineKind::JIT)
#ifndef NDEBUG
                .setVerifyModules(true)
#endif
                // .setMCJITMemoryManager(llvm::make_unique<llvm::SectionMemoryManager>())
                               .create());
        if (!_executionEngine.get()) {
            throw CGException("Could not create ExecutionEngine: ", errStr);
        }

        _fpm.reset(new llvm::legacy::FunctionPassManager(_module));

        preparePassManager();

        _fpm->doInitialization();

        /**
         *
         */
        this->validate();
    }

    LlvmModelLibrary4_0(const LlvmModelLibrary4_0&) = delete;
    LlvmModelLibrary4_0& operator=(const LlvmModelLibrary4_0&) = delete;

    inline virtual ~LlvmModelLibrary4_0() {
        this->cleanUp();
    }

    /**
     * Set up the optimizer pipeline
     */
    virtual void preparePassManager() {
        llvm::PassManagerBuilder builder;
        builder.OptLevel = 2;
        builder.populateFunctionPassManager(*_fpm);
        //_fpm.add(new DataLayoutPass());
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
        llvm::raw_os_ostream os(std::cerr);
        bool failed = llvm::verifyFunction(*func, &os);
        if (failed)
            throw CGException("Function '", functionName, "' verification failed");
#endif

        // Optimize the function.
        _fpm->run(*func);

        // JIT the function, returning a function pointer.
        uint64_t fPtr = _executionEngine->getFunctionAddress(functionName);
        if (fPtr == 0 && required) {
            throw CGException("Unable to find function '", functionName, "' in LLVM module");
        }
        return (void*) fPtr;
    }

    friend class LlvmModel<Base>;

};

} // END cg namespace
} // END CppAD namespace

#endif
