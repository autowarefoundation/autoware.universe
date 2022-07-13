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

TEST_F(CppADCGSolveTest, SolveMul) {
    // independent variable vector
    std::vector<ADCGD> u(2);
    u[0] = 4.0;
    u[1] = 2.0;

    Independent(u);

    // dependent variable vector
    std::vector<ADCGD> Z(4);

    // model
    Z[0] = u[0] * u[1];
    Z[1] = Z[0] * 4.;
    Z[2] = 2. * Z[1];
    Z[3] = Z[2] * 1 - 64.0;

    // create f: U -> Z
    ADFun<CGD> fun(u, Z);

    test_solve(fun, 3, 0, u);
    test_solve(fun, 3, 1, u);
}

