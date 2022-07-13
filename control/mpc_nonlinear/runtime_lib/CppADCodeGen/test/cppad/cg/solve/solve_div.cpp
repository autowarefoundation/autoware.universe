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
#include "CppADCGSolveTest.hpp"

using namespace CppAD;
using namespace CppAD::cg;

TEST_F(CppADCGSolveTest, SolveDiv) {
    //verbose_ = true; // this will print the solutions

    // independent variable vector
    std::vector<ADCGD> x(2);
    x[0] = 4.0;
    x[1] = 1.0;

    Independent(x);

    // dependent variable vector
    std::vector<ADCGD> y(4);

    // model
    y[0] = x[0] / (x[1] + 1);
    y[1] = y[0] / 4.;
    y[2] = 2. / y[1];
    y[3] = y[2] / 1 - 4.0; // 2. / (x[0] / (x[1] + 1) / 4.) - 4.0 == 0

    // create f: x -> y
    ADFun<CGD> fun(x, y);

    test_solve(fun, 3, 0, x);
    test_solve(fun, 3, 1, x);
}
