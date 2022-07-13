#ifndef CPPAD_CG_DOT_UTIL_INCLUDED
#define CPPAD_CG_DOT_UTIL_INCLUDED
/* --------------------------------------------------------------------------
 *  CppADCodeGen: C++ Algorithmic Differentiation with Source Code Generation:
 *    Copyright (C) 2016 Ciengis
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

#include <cppad/cg/lang/c/lang_c_default_var_name_gen.hpp>

namespace CppAD {
namespace cg {

/**
 * Prints the graph resulting in a single dependent variable.
 */
template<class Base>
inline void printDotExpression(CG<Base>& dep,
                               std::ostream& out = std::cout) {
    if (dep.getOperationNode() != nullptr) {
        if (dep.getOperationNode()->getCodeHandler() == nullptr) {
            throw CGException("Unable to print expression: found an operation node without a CodeHandler!");
        }

        CodeHandler<Base>& handler = *dep.getOperationNode()->getCodeHandler();
        LanguageDot<double> langDot;
        LangCDefaultVariableNameGenerator<double> nameGen;

        std::vector<CG<Base> > depv(1);
        depv[0] = dep;

        std::ostringstream code;
        handler.generateCode(code, langDot, depv, nameGen);
        out << code.str();
    } else {
        out << "digraph {\n"
                "\"" << dep.getValue() << "\" -> \"y[0]\"\n"
                "}" << std::endl;
    }
}

template<class Base>
inline void printDotExpression(OperationNode<Base>& dep,
                               std::ostream& out = std::cout) {
    printDotExpression(CG<Base>(dep), out);
}

}
}

#endif