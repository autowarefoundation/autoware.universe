#ifndef CPPAD_CG_MODEL_C_SOURCE_GEN_FOR0_INCLUDED
#define CPPAD_CG_MODEL_C_SOURCE_GEN_FOR0_INCLUDED
/* --------------------------------------------------------------------------
 *  CppADCodeGen: C++ Algorithmic Differentiation with Source Code Generation:
 *    Copyright (C) 2012 Ciengis
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
void ModelCSourceGen<Base>::generateZeroSource() {
    const std::string jobName = "model (zero-order forward)";

    startingJob("'" + jobName + "'", JobTimer::GRAPH);

    CodeHandler<Base> handler;
    handler.setJobTimer(_jobTimer);

    std::vector<CGBase> indVars(_fun.Domain());
    handler.makeVariables(indVars);
    if (_x.size() > 0) {
        for (size_t i = 0; i < indVars.size(); i++) {
            indVars[i].setValue(_x[i]);
        }
    }

    std::vector<CGBase> dep;

    if (_loopTapes.empty()) {
        dep = _fun.Forward(0, indVars);
    } else {
        /**
         * Contains loops
         */
        dep = prepareForward0WithLoops(handler, indVars);
    }

    finishedJob();

    LanguageC<Base> langC(_baseTypeName);
    langC.setMaxAssignmentsPerFunction(_maxAssignPerFunc, &_sources);
    langC.setMaxOperationsPerAssignment(_maxOperationsPerAssignment);
    langC.setParameterPrecision(_parameterPrecision);
    langC.setGenerateFunction(_name + "_" + FUNCTION_FORWAD_ZERO);

    std::ostringstream code;
    std::unique_ptr<VariableNameGenerator<Base> > nameGen(createVariableNameGenerator());

    handler.generateCode(code, langC, dep, *nameGen, _atomicFunctions, jobName);
}


} // END cg namespace
} // END CppAD namespace

#endif