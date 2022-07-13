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

TEST_F(CppADCGSolveTest, SolveSub) {
    // independent variable vector
    std::vector<ADCGD> u(2);
    u[0] = 4.5;
    u[1] = 2.5;
    Independent(u);

    // dependent variable vector
    std::vector<ADCGD> Z(3);

    // model
    Z[0] = u[0] - u[1]; // AD<double> + AD<double>
    Z[1] = Z[0] - 1.; // AD<double> + double
    Z[2] = 1. - Z[1]; // double + AD<double> 

    // create f: U -> Z 
    ADFun<CGD> fun(u, Z);

    test_solve(fun, 2, 1, u);
}

TEST_F(CppADCGSolveTest, SolveSubInvalid) {
    // independent variable vector
    std::vector<ADCGD> u(1);
    u[0] = 4.5;
    Independent(u);

    // dependent variable vector
    std::vector<ADCGD> Z(1);

    // model
    Z[0] = u[0] - u[0]; // u[0] disappears

    // create f: U -> Z
    ADFun<CGD> fun(u, Z);

    ASSERT_THROW(test_solve(fun, 0, 0, u), CGException);
}
