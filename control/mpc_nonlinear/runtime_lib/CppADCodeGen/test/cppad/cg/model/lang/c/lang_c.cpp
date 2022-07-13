/* --------------------------------------------------------------------------
 *  CppADCodeGen: C++ Algorithmic Differentiation with Source Code Generation:
 *    Copyright (C) 2019 Ciengis
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
#include <fstream>

#include "CppADCGTest.hpp"
#include <cppad/cg/cppadcg.hpp>
#include <cppad/cg/lang/dot/dot.hpp>
#include <cppad/cg/lang/c/lang_c_default_var_name_gen.hpp>
#include <gtest/gtest.h>

namespace CppAD {
namespace cg {

class CppADCGTestLangC : public CppADCGTest {
protected:
    using Base = double;
    using CGD = CppAD::cg::CG<Base>;
    using ADCG = CppAD::AD<CGD>;
public:

    inline explicit CppADCGTestLangC(bool verbose = false,
                                     bool printValues = false) :
            CppADCGTest(verbose, printValues) {
    }

    void testNumberOfSources(size_t maxAssignPerFunction,
                             size_t maxOperationsPerAssign,
                             size_t expectedNumberOfSources) {
        ADFun<CGD> fun = model();

        /**
         * start the special steps for source code generation
         */
        CodeHandler<double> handler;

        CppAD::vector<CGD> indVars(5);
        handler.makeVariables(indVars);

        CppAD::vector<CGD> vals = fun.Forward(0, indVars);

        LanguageC<double> langC("double");
        LangCDefaultVariableNameGenerator<double> nameGen;

        std::ostringstream code;

        std::map<std::string, std::string> sources;
        langC.setMaxAssignmentsPerFunction(maxAssignPerFunction, &sources);
        langC.setGenerateFunction("split_model");
        langC.setMaxOperationsPerAssignment(maxOperationsPerAssign);

        handler.generateCode(code, langC, vals, nameGen);

        if (this->verbose_) {
            printSources(sources);
        }

        ASSERT_EQ(sources.size(), expectedNumberOfSources);
    }

protected:
    inline static ADFun<CGD> model() {
        // independent variable vector
        CppAD::vector<ADCG> x(5);
        x[0] = 2.;
        x[1] = 3.;
        Independent(x);

        // dependent variable vector
        CppAD::vector<ADCG> y(4);

        // the model
        //file 1:
        ADCG a = x[0] / 1. + x[1] * x[1] + x[2] * x[2] + x[3] * x[3] + x[4] * x[4];
        ADCG b = a / 2e-6;

        //file 2:
        y[0] = b + 1 / (sign(b) * 5 * a);
        y[1] = b + pow(a, 2 * b);

        //file 3:
        y[2] = b + pow(3 * a, 2.0);

        // main file
        y[3] = x[1];

        ADFun<CGD> fun(x, y); // the model tape

        return fun;
    }

    inline static void printSources(const std::map<std::string, std::string>& sources) {
        for (const auto& name2content : sources) {
            std::ofstream texfile;
            texfile.open(name2content.first);
            texfile << name2content.second;
        }
    }
};

}
}

using namespace CppAD;
using namespace CppAD::cg;


TEST_F(CppADCGTestLangC, maxAssignmentPerFunc) {

    testNumberOfSources(2u,
                        (std::numeric_limits<size_t>::max)(),
                        4u);
}

TEST_F(CppADCGTestLangC, maxOperationPerAssign2) {

    testNumberOfSources(2u,
                        2u,
                        7u);
}

TEST_F(CppADCGTestLangC, maxOperationPerAssign1) {

    testNumberOfSources(2u,
                        1u,
                        11u);
}