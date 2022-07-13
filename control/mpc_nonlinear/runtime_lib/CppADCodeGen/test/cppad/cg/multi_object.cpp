/* --------------------------------------------------------------------------
 *  CppADCodeGen: C++ Algorithmic Differentiation with Source Code Generation:
 *    Copyright (C) 2020 Joao Leal
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

#include <cppad/cg.hpp>
#include <cppad/cg/lang/dot/dot.hpp>
#include <cppad/cg/lang/latex/latex.hpp>
#include <cppad/cg/lang/mathml/mathml.hpp>

#include "multi_object.hpp"

using namespace CppAD;
using namespace CppAD::cg;
using namespace std;

void callCodeGen2() {
    CodeHandler<double> handler;

    size_t m = 3;
    size_t n = 3;

    // independent variables of CppAD
    std::vector<CG<double> > x(n);
    handler.makeVariables(x);

    // independent variables of CppAD
    std::vector<AD<CG<double> > > ax(n);
    ax[0] = x[0];
    ax[1] = x[1];
    ax[2] = x[2];

    CppAD::Independent(ax);

    // dependent variable of CppAD
    std::vector<AD<CG<double> > > ay(m);
    ay[0] = x[0] + 2.0;
    ay[1] = x[1] + 3.0;
    ay[2] = x[2] * 4.0;

    CppAD::ADFun<CG<double> > f(ax, ay);

    std::vector<CG<double> > dep = f.Forward(0, x);

    {
        LanguageC<double> languageC("double");
        LangCDefaultVariableNameGenerator<double> nameGen;

        ostringstream code;
        handler.generateCode(code, languageC, dep, nameGen);

    }

    {
        LanguageDot<double> languageDot;
        LangCDefaultVariableNameGenerator<double> nameGen;

        ostringstream code;
        handler.generateCode(code, languageDot, dep, nameGen);
    }

    {
        LanguageLatex<double> languageLatex;
        LangLatexDefaultVariableNameGenerator<double> nameGen;

        ostringstream code;
        handler.generateCode(code, languageLatex, dep, nameGen);
    }

    {
        LanguageMathML<double> languageMathml;
        LangMathMLDefaultVariableNameGenerator<double> nameGen;

        ostringstream code;
        handler.generateCode(code, languageMathml, dep, nameGen);
    }

}