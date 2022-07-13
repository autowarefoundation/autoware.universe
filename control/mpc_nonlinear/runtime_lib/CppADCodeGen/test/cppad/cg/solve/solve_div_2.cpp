/* --------------------------------------------------------------------------
 *  CppADCodeGen: C++ Algorithmic Differentiation with Source Code Generation:
 *    Copyright (C) 2016 Ciengis
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

/**
 * @test 2 instances of the save variable with 1 bifurcation
 */
TEST_F(CppADCGSolveTest, SolveDiv_2var_1bif) {
    this->verbose_ = true;

    // independent variable vector
    std::vector<ADCGD> x(2);
    x[0] = -1.5;
    x[1] = -0.25 / 2;
    Independent(x);

    // dependent variable vector
    std::vector<ADCGD> y(3);

    // model
    y[0] = x[0] + x[1] / 0.5 + x[1] / 0.5;
    y[1] = y[0] + 1.;
    y[2] = 1. + y[1];

    // create f: x -> y
    ADFun<CGD> fun(x, y);

    size_t eqIdx = 2;
    size_t varIdx = 1;
    test_solve(fun, eqIdx, varIdx, x);
}

/**
 * @test 2 instances of the save variable with 2 bifurcations
 */
TEST_F(CppADCGSolveTest, SolveDiv_2var_2bif) {
    this->verbose_ = true;

    // independent variable vector
    std::vector<ADCGD> x(2);
    x[0] = -1.5;
    x[1] = -0.25 / 2;
    Independent(x);

    // dependent variable vector
    std::vector<ADCGD> y(3);

    // model
    y[0] = x[0] + x[1] / 0.5 + x[1] / 0.5;
    y[1] = y[0] / 2 + 1. + y[0] / 2;
    y[2] = 1. + y[1];

    // create f: x -> y
    ADFun<CGD> fun(x, y);

    size_t eqIdx = 2;
    size_t varIdx = 1;
    test_solve(fun, eqIdx, varIdx, x);
}

/**
 * @test 2 instances of the save variable with 2 bifurcations
 *         (also includes multiplications)
 */
TEST_F(CppADCGSolveTest, SolveDiv_2var_2bif_alt) {
    this->verbose_ = true;

    // independent variable vector
    std::vector<ADCGD> x(2);
    x[0] = -1.5;
    x[1] = -0.25 / 2;
    Independent(x);

    // dependent variable vector
    std::vector<ADCGD> y(3);

    // model
    y[0] = x[0] + 2 * x[1] + 2 * x[1];
    y[1] = y[0] / 2 + 1. + y[0] / 2;
    y[2] = 1. + y[1];

    // create f: x -> y
    ADFun<CGD> fun(x, y);

    size_t eqIdx = 2;
    size_t varIdx = 1;
    test_solve(fun, eqIdx, varIdx, x);
}