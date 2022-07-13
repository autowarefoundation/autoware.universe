/* --------------------------------------------------------------------------
 *  CppADCodeGen: C++ Algorithmic Differentiation with Source Code Generation:
 *    Copyright (C) 2018 Ciengis
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
#include "cppad/cg/evaluator/CppADCGEvaluatorTest.hpp"

using namespace CppAD;
using namespace CppAD::cg;

TEST_F(CppADCGEvaluatorTest, Print) {
    ModelType model = [&](const std::vector<CGD>& x) {
        // dependent variable vector
        std::vector<CGD> y(4);

        CGD a(5);

        y[0] = makePrintValue("x[0] = ", x[0], "\n");
        y[1] = makePrintValue("y[1] = ", x[0] + x[1], "\n");
        y[2] = makePrintValue("y[2] = ", y[0], "\n");
        y[3] = makePrintValue("y[3] = ", a, "\n");

        return y;
    };

    this->testCG(model, std::vector<double>{0.5, 1.5});
}
