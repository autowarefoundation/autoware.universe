#ifndef CPPAD_CG_LLVM_MODEL_INCLUDED
#define CPPAD_CG_LLVM_MODEL_INCLUDED
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

/**
 * Useful class to call the JIT'ed models with LLVM.
 *
 * @author Joao Leal
 */
template<class Base>
class LlvmModel : public FunctorGenericModel<Base> {
protected:
    /// the dynamic library
    LlvmModelLibrary<Base>* _dynLib;

public:

    LlvmModel(const LlvmModel&) = delete;
    LlvmModel& operator=(const LlvmModel&) = delete;

    virtual ~LlvmModel() {
        if (_dynLib != nullptr) {
            _dynLib->destroyed(this);
        }
    }

protected:

    /**
     * Creates a new model
     *
     * @param name The model name
     */
    LlvmModel(LlvmModelLibrary<Base>* dynLib,
              const std::string& name) :
        FunctorGenericModel<Base>(name),
        _dynLib(dynLib) {

        CPPADCG_ASSERT_UNKNOWN(_dynLib != nullptr);

        this->init();
    }

    void* loadFunction(const std::string& functionName, bool required = true) override {
        return _dynLib->loadFunction(functionName, required);
    }

    void modelLibraryClosed() override {
        _dynLib = nullptr;
        FunctorGenericModel<Base>::modelLibraryClosed();
    }

    friend class LlvmModelLibrary<Base>;
};

} // END cg namespace
} // END CppAD namespace

#endif
