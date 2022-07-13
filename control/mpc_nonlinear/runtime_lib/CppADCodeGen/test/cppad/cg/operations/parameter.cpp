/* --------------------------------------------------------------------------
 *  CppADCodeGen: C++ Algorithmic Differentiation with Source Code Generation:
 *    Copyright (C) 2012 Ciengis
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
#include "CppADCGOperationTest.hpp"
#include "parameter.hpp"

using namespace CppAD;
using namespace CppAD::cg;

TEST_F(CppADCGOperationTest, parameter) {
    // number of different parameter values
    size_t n_parameter = 7;

    // number of parameter repeats
    size_t n_repeat = 5;

    // independent variable vector
    size_t n = n_parameter * n_repeat;
    std::vector<double> u(n);
    for (size_t j = 0; j < n; j++) {
        u[j] = double(j);
    }

    test0nJac("parameter", &ParameterFunc<double >, &ParameterFunc<CG<double> >, u);
}

