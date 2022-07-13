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
#include <cmath>

#include "CppADCGEvaluatorAdolcTest.hpp"

using namespace CppAD;
using namespace CppAD::cg;

TEST_F(CppADCGEvaluatorAdolcTest, SolvePow) {
    ModelType model = [](const std::vector<CGD>& x) {
        std::vector<CGD> y(4);

        // dependent variables
        y[0] = pow(x[0], x[1]) + 16.0;
        y[1] = y[0] - x[2] * pow(4, 2);
        y[2] = pow(y[0], 2.0) - pow(pow(x[3], 2), 2);
        y[3] = pow(2, x[1]) + pow(x[1], 2);
        return y;
    };

    this->test(model, std::vector<double>{4.0, 2.0, 1.0, 4.0});
}
