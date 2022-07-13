/* --------------------------------------------------------------------------
 *  CppADCodeGen: C++ Algorithmic Differentiation with Source Code Generation:
 *    Copyright (C) 2014 Ciengis
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
#include <cppad/cg/lang/latex/latex.hpp>
#include <gtest/gtest.h>

using namespace CppAD;
using namespace CppAD::cg;

TEST(CppADCGLatexTest, latex) { // NOLINT(cert-err58-cpp)
    // use a special object for source code generation
    using CGD = CG<double>;
    using ADCG = AD<CGD>;

    // independent variable vector
    CppAD::vector<ADCG> x(2);
    x[0] = 2.;
    x[1] = 3.;
    Independent(x);

    // dependent variable vector 
    CppAD::vector<ADCG> y(4);

    // the model
    ADCG a = x[0] / 1. + x[1] * x[1];
    ADCG b = a / 2e-6;
    y[0] = b + 1 / (sign(b)*5 * a);
    y[1] = b + pow(a, 2 * b);
    y[2] = b + pow(3 * a, 2.0);
    y[3] = x[1];

    ADFun<CGD> fun(x, y); // the model tape

    /**
     * start the special steps for source code generation
     * for a Jacobian
     */
    CodeHandler<double> handler;

    CppAD::vector<CGD> indVars(2);
    handler.makeVariables(indVars);

    //CppAD::vector<CGD> jac = fun.SparseJacobian(indVars);
    CppAD::vector<CGD> vals = fun.Forward(0, indVars);

    LanguageLatex<double> langLatex;
    LangLatexDefaultVariableNameGenerator<double> nameGen;

    std::ofstream texfile;
    texfile.open("algorithm.tex");

    handler.generateCode(texfile, langLatex, vals, nameGen);

    texfile.close();

    std::string dir = system::getWorkingDirectory();

    ASSERT_NO_THROW(system::callExecutable(PDFLATEX_COMPILER,{"-halt-on-error", "-interaction=nonstopmode", "-shell-escape", system::createPath(dir, "latexTemplate.tex")}));

}


TEST(CppADCGLatexTest, latexJac) { // NOLINT(cert-err58-cpp)
    // use a special object for source code generation
    using CGD = CG<double>;
    using ADCG = AD<CGD>;

    // independent variable vector
    CppAD::vector<ADCG> x(4);
    x[0] = 2.;
    Independent(x);

    // dependent variable vector 
    CppAD::vector<ADCG> y(1);

    // the model
    y[0] = x[0] * x[1] * x[2] * x[3];

    ADFun<CGD> fun(x, y); // the model tape

    /**
     * start the special steps for source code generation
     * for a Jacobian
     */
    CodeHandler<double> handler;

    CppAD::vector<CGD> indVars(x.size());
    handler.makeVariables(indVars);

    CppAD::vector<CGD> jac = fun.SparseJacobian(indVars);
    //CppAD::vector<CGD> vals = fun.Forward(0, indVars);

    LanguageLatex<double> langLatex;
    LangLatexDefaultVariableNameGenerator<double> nameGen;

    std::ofstream texfile;
    texfile.open("algorithm.tex");

    handler.generateCode(texfile, langLatex, jac, nameGen);

    texfile.close();

    std::string dir = system::getWorkingDirectory();

    ASSERT_NO_THROW(system::callExecutable(PDFLATEX_COMPILER, {"-halt-on-error", "-interaction=nonstopmode", "-shell-escape", system::createPath(dir, "latexTemplate.tex")}));

}