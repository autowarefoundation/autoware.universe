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
//#include <cmath>

#include "CppADCGEvaluatorTest.hpp"

using namespace CppAD;
using namespace CppAD::cg;

TEST_F(CppADCGEvaluatorTest, Log10) {
    ModelType model = [](const std::vector<CGD>& x) {
        std::vector<CGD> y(2);

        y[0] = CppAD::log10(x[0]);
        y[1] = CppAD::log10(y[0]) - x[1] * CppAD::log10(CppAD::log10(11.0));
        return y;
    };

    this->test(model, std::vector<double>{11.0, 1.0});
}


