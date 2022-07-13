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

#include "CppADCGTest.hpp"
#include <cppad/cg/lang/mathml/mathml.hpp>

using namespace CppAD;
using namespace CppAD::cg;

TEST_F(CppADCGTest, HandlerReset) {
    using namespace CppAD;
    using namespace std;

    CodeHandler<double> handler(20);

    // independent variables of CppAD
    std::vector<CG<double> > x(3);
    handler.makeVariables(x);

    // independent variables of CppAD
    std::vector<AD<CG<double> > > ax(3);
    ax[0] = x[0];
    ax[1] = x[1];
    ax[2] = x[2];

    CppAD::Independent(ax);

    // dependent variable of CppAD
    std::vector<AD<CG<double> > > ay(3);
    ay[0] = x[0] + 2.0;
    ay[1] = x[1] + 3.0;
    ay[2] = x[2] * 4.0;

    CppAD::ADFun<CG<double> > f(ax, ay);

    std::vector<CG<double> > dep = f.Forward(0, x);

    LanguageMathML<double> langMathML;
    LangMathMLDefaultVariableNameGenerator<double> nameGen;

    ostringstream code;
    handler.generateCode(code, langMathML, dep, nameGen);
    string code1 = code.str();

    code.str("");
    handler.generateCode(code, langMathML, dep, nameGen);
    string code2 = code.str();

    ASSERT_EQ(code1, code2);
}
