/* --------------------------------------------------------------------------
 *  CppADCodeGen: C++ Algorithmic Differentiation with Source Code Generation:
 *    Copyright (C) 2014 Ciengis
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
#include "CppADCGEvaluatorAdolcTest.hpp"

using namespace CppAD;
using namespace CppAD::cg;

TEST_F(CppADCGEvaluatorAdolcTest, SolveMul) {
    ModelType model = [](const std::vector<CGD>& x) {
        std::vector<CGD> y(4);

        // model
        y[0] = x[0] * x[1];
        y[1] = y[0] * 4.;
        y[2] = 2. * y[1];
        y[3] = y[2] * 1 - 64.0;
        return y;
    };

    this->test(model, std::vector<double>{4.0, 2.0});
}

