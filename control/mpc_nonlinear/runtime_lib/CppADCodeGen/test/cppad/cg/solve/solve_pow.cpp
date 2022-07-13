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
#include <cmath>

#include "CppADCGSolveTest.hpp"

using namespace CppAD;
using namespace CppAD::cg;

TEST_F(CppADCGSolveTest, SolvePow) {
    // independent variable vector
    std::vector<ADCGD> u(4);
    u[0] = 4.0;
    u[1] = 2.0;
    u[2] = 1.0;
    u[3] = 4.0;

    Independent(u);

    // dependent variable vector
    std::vector<ADCGD> Z(3);

    // dependent variables
    Z[0] = pow(u[0], u[1]) - 16.0;
    Z[1] = Z[0] - u[2] * pow(4, 2);
    Z[2] = pow(Z[0], 2.0) - pow(pow(u[3], 2), 2);

    // create f: U -> Z
    ADFun<CGD> fun(u, Z);

    // test_solve(fun, 2, 0, u);// <<< should fail
    test_solve(fun, 0, 1, u);
}
