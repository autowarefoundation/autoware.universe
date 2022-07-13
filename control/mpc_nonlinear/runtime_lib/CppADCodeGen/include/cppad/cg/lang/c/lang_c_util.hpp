#ifndef CPPAD_CG_LANG_C_UTIL_INCLUDED
#define CPPAD_CG_LANG_C_UTIL_INCLUDED
/* --------------------------------------------------------------------------
 *  CppADCodeGen: C++ Algorithmic Differentiation with Source Code Generation:
 *    Copyright (C) 2016 Ciengis
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

namespace CppAD {
namespace cg {

/**
 * Prints the model to standard output
 * 
 * @param fun the model
 */
template<class Base>
inline void printModel(ADFun<CG<Base> >& fun) {
    std::vector<std::string> depNames;
    std::vector<std::string> indepNames;
    printModel(fun, depNames, indepNames);
}

/**
 * Prints the model to standard output
 * 
 * @param fun the model
 * @param depNames the names to be used for the dependent variables
 * @param indepNames the names to be used for the independent variables
 */
template<class Base>
inline void printModel(ADFun<CG<Base> >& fun,
                       const std::vector<std::string>& depNames,
                       const std::vector<std::string>& indepNames) {
    CPPADCG_ASSERT_UNKNOWN(depNames.size() <= fun.Range())
    CPPADCG_ASSERT_UNKNOWN(indepNames.size() <= fun.Domain())

    CodeHandler<Base> handler;

    std::vector<CG<Base> > indep0(fun.Domain());
    handler.makeVariables(indep0);

    std::vector<CG<Base> > dep0 = fun.Forward(0, indep0);

    LanguageC<Base> langC("double");

    /**
     * generate the source code
     */
    LangCCustomVariableNameGenerator<Base> nameGen(depNames, indepNames,
                                                   "y", "x", "z", "array");

    std::ostringstream code;
    handler.generateCode(code, langC, dep0, nameGen);
    std::cout << "\n" << code.str() << std::endl;
}

/**
 * Prints the model resulting in a single dependent variable.
 */
template<class Base>
inline void printExpression(const CG<Base>& dep,
                            std::ostream& out = std::cout) {
    if(dep.getOperationNode() != nullptr) {
        if(dep.getOperationNode()->getCodeHandler() == nullptr) {
            throw CGException("Unable to print expression: found an operation node without a CodeHandler!");
        }

        CodeHandler<Base>& handler = *dep.getOperationNode()->getCodeHandler();
        LanguageC<double> langC("double");
        LangCDefaultVariableNameGenerator<double> nameGen;

        std::vector<CG<Base> > depv(1);
        depv[0] = dep;

        std::ostringstream code;
        handler.generateCode(code, langC, depv, nameGen);
        out << code.str();
    } else {
        out << "y[0] = " << dep.getValue() << ";" << std::endl;
    }
}

template<class Base>
inline void printExpression(OperationNode<Base>& dep,
                            std::ostream& out = std::cout) {
    printExpression(CG<Base>(dep), out);
}

} // END cg namespace
} // END CppAD namespace

#endif