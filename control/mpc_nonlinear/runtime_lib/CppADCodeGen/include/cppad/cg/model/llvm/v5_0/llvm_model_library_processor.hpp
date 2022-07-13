#ifndef CPPAD_CG_LLVM_MODEL_LIBRARY_PROCESSOR_INCLUDED
#define CPPAD_CG_LLVM_MODEL_LIBRARY_PROCESSOR_INCLUDED
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

#include <cppad/cg/model/llvm/v5_0/llvm_base_model_library_processor_impl.hpp>

namespace CppAD {
namespace cg {

/**
 * Useful class for generating a JIT evaluated model library.
 *
 * @author Joao Leal
 */
template<class Base>
class LlvmModelLibraryProcessor : public LlvmBaseModelLibraryProcessorImpl<Base> {
public:

    /**
     * Creates a LLVM model library processor.
     *
     * @param librarySourceGen
     */
    LlvmModelLibraryProcessor(ModelLibraryCSourceGen<Base>& librarySourceGen) :
        LlvmBaseModelLibraryProcessorImpl<Base>(librarySourceGen, "5.0") {
    }

    virtual ~LlvmModelLibraryProcessor() = default;

    using LlvmBaseModelLibraryProcessorImpl<Base>::create;

    static inline std::unique_ptr<LlvmModelLibrary<Base>> create(ModelLibraryCSourceGen<Base>& modelLibraryHelper) {
        LlvmModelLibraryProcessor<Base> p(modelLibraryHelper);
        return p.create();
    }

};

} // END cg namespace
} // END CppAD namespace

#endif
