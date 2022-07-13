/* --------------------------------------------------------------------------
 *  CppADCodeGen: C++ Algorithmic Differentiation with Source Code Generation:
 *    Copyright (C) 2015 Ciengis
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

#include <iostream>

#include <cppad/cg/cppadcg.hpp>
#include <cppad/cg/lang/mathml/mathml.hpp>
#include <gtest/gtest.h>

using namespace CppAD;
using namespace CppAD::cg;

TEST(CppADCGLatexTest, latex) {
    // use a special object for source code generation
    using CGD = CG<double>;
    using ADCG = AD<CGD>;

    // independent variable vector
    CppAD::vector<ADCG> x(2);
    x[0] = 2.;
    x[1] = 3.;
    Independent(x);

    // dependent variable vector 
    CppAD::vector<ADCG> y(8);

    // the model
    ADCG a = x[0] / 1. + x[1] * x[1];
    ADCG b = a / 2e-6;
    y[0] = b + 1 / (sign(b) * 5 * a);
    y[1] = x[1];
    y[2] = CondExpLt(ADCG(1.0), x[0], x[1], b);
    y[3] = CondExpLe(x[0], ADCG(2.0), x[1], b);
    y[4] = CondExpEq(x[0], x[1], x[1], b);
    ADCG c = CondExpGe(ADCG(3.0), x[0], a, b);
    y[5] = CondExpGt(ADCG(4.0), x[0], ADCG(5.0), c);
    y[6] = 5 * pow(4, x[0]);
    y[7] = 3;

    ADFun<CGD> fun(x, y); // the model tape

    /**
     * start the special steps for source code generation
     */
    CodeHandler<double> handler;

    CppAD::vector<CGD> indVars(2);
    handler.makeVariables(indVars);

    CppAD::vector<CGD> vals = fun.Forward(0, indVars);

    LanguageMathML<double> langMathML;
    LangMathMLDefaultVariableNameGenerator<double> nameGen;

    langMathML.setSaveVariableRelations(true);

    // add some additional code to select variables
    langMathML.setStyle(langMathML.getStyle() + readStringFromFile("mathml.css"));

    // use block display
    langMathML.setEquationMarkup(R"(<math display="block" class="equation">)", "</math>");

    // use inline display
    //langMathML.setEquationMarkup("<math>", "</math><br/>");

    // use MathJax (and align to the left)
    langMathML.setHeadExtraMarkup(readStringFromFile("head_extra.html"));

    langMathML.setJavascript(readStringFromFile("variableSelection.js"));

    // create the HMTL file
    std::ofstream htmlFile;
    htmlFile.open("algorithm.html");

    handler.setReuseVariableIDs(false);
    handler.generateCode(htmlFile, langMathML, vals, nameGen);

    htmlFile.close();
}
