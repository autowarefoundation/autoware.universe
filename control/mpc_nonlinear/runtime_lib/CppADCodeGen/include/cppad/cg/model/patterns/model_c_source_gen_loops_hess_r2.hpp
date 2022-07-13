#ifndef CPPAD_CG_MODEL_C_SOURCE_GEN_LOOPS_HESS_R2_INCLUDED
#define CPPAD_CG_MODEL_C_SOURCE_GEN_LOOPS_HESS_R2_INCLUDED
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

namespace CppAD {
namespace cg {

template<class Base>
void ModelCSourceGen<Base>::generateSparseHessianWithLoopsSourceFromRev2(const std::map<size_t, CompressedVectorInfo>& hessInfo,
                                                                         size_t maxCompressedSize) {
    using namespace std;
    using namespace CppAD::cg::loops;

    startingJob("'sparse Hessian'", JobTimer::SOURCE_GENERATION);

    /**
     * Generate the source code
     */
    LanguageC<Base> langC(_baseTypeName);
    string argsDcl = langC.generateDefaultFunctionArgumentsDcl();

    string model_function = _name + "_" + FUNCTION_SPARSE_HESSIAN;
    string functionRev2 = _name + "_" + FUNCTION_SPARSE_REVERSE_TWO;
    string suffix = "indep";
    string nlRev2Suffix = "noloop_" + suffix;

    _cache.str("");
    _cache << "#include <stdlib.h>\n"
            << LanguageC<Base>::ATOMICFUN_STRUCT_DEFINITION << "\n\n";
    generateFunctionDeclarationSource(_cache, functionRev2, nlRev2Suffix, _nonLoopRev2Elements, argsDcl);
    generateFunctionDeclarationSourceLoopForRev(_cache, langC, _name, "jrow", _loopRev2Groups, generateFunctionNameLoopRev2);

    _cache << "\n";

    printForRevUsageFunction(_cache, _baseTypeName, _name,
                             model_function, 3,
                             functionRev2, suffix,
                             "jrow", "it", "hess",
                             _loopRev2Groups,
                             _nonLoopRev2Elements,
                             hessInfo,
                             generateFunctionNameLoopRev2,
                             _hessSparsity.rows.size(), maxCompressedSize);

    finishedJob();

    _sources[model_function + ".c"] = _cache.str();
    _cache.str("");
}

} // END cg namespace
} // END CppAD namespace

#endif