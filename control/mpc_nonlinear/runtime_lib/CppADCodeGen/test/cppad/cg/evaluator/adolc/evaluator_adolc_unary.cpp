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

TEST_F(CppADCGEvaluatorAdolcTest, SolveUnaryMinus) {
    ModelType model = [](const std::vector<CGD>& x) {
        std::vector<CGD> y(3);

        // model
        y[0] = -x[1] + x[0]; // AD<double> + AD<double>
        y[1] = -y[0] + 1.; // AD<double> + double
        y[2] = 1. + y[1]; // double + AD<double> 
        return y;
    };

    this->test(model, std::vector<double>{4.5, 2.5});
}
