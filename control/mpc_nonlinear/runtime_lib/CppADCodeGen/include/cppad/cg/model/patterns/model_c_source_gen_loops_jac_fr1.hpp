#ifndef CPPAD_CG_MODEL_C_SOURCE_GEN_LOOPS_JAC_FR1_INCLUDED
#define CPPAD_CG_MODEL_C_SOURCE_GEN_LOOPS_JAC_FR1_INCLUDED
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
void ModelCSourceGen<Base>::generateSparseJacobianWithLoopsSourceFromForRev(const std::map<size_t, CompressedVectorInfo>& jacInfo,
                                                                            size_t maxCompressedSize,
                                                                            const std::string& localFunctionTypeName,
                                                                            const std::string& suffix,
                                                                            const std::string& keyName,
                                                                            const std::map<size_t, std::set<size_t> >& nonLoopElements,
                                                                            const std::map<LoopModel<Base>*, std::map<size_t, std::map<size_t, std::set<size_t> > > >& loopGroups,
                                                                            void (*generateLocalFunctionName)(std::ostringstream& cache, const std::string& modelName, const LoopModel<Base>& loop, size_t g)) {
    using namespace std;
    using namespace CppAD::cg::loops;

    startingJob("'sparse Jacobian'", JobTimer::SOURCE_GENERATION);

    /**
     * Generate the source code
     */
    LanguageC<Base> langC(_baseTypeName);
    string argsDcl = langC.generateDefaultFunctionArgumentsDcl();

    string model_function = _name + "_" + FUNCTION_SPARSE_JACOBIAN;
    string localFunction = _name + "_" + localFunctionTypeName;
    string nlSuffix = "noloop_" + suffix;

    _cache.str("");
    _cache << "#include <stdlib.h>\n"
            << LanguageC<Base>::ATOMICFUN_STRUCT_DEFINITION << "\n\n";

    generateFunctionDeclarationSource(_cache, localFunction, nlSuffix, nonLoopElements, argsDcl);
    generateFunctionDeclarationSourceLoopForRev(_cache, langC, _name, keyName, loopGroups, generateLocalFunctionName);

    _cache << "\n";
    printForRevUsageFunction(_cache, _baseTypeName, _name,
            model_function, 2,
            localFunction, suffix,
            keyName, "it", "jac",
            loopGroups,
            nonLoopElements,
            jacInfo,
            generateLocalFunctionName,
            _jacSparsity.rows.size(), maxCompressedSize);

    finishedJob();

    _sources[model_function + ".c"] = _cache.str();
    _cache.str("");
}

} // END cg namespace
} // END CppAD namespace

#endif