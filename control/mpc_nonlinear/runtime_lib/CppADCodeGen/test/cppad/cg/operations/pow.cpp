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
#include "pow.hpp"

using namespace CppAD;
using namespace CppAD::cg;

TEST_F(CppADCGOperationTest, PowTestOne) {
    std::vector<double> u{0.5, 2.0}; // domain space vector

    test0nJac("PowTestOne", &PowTestOneFunc<double >, &PowTestOneFunc<CG<double> >, u);
}

TEST_F(CppADCGOperationTest, PowTestTwo) {
    std::vector<double> u{2., 3.}; // domain space vector

    test0nJac("PowTestTwo", &PowTestTwoFunc<double >, &PowTestTwoFunc<CG<double> >, u);
}

TEST_F(CppADCGOperationTest, PowTestThree) {
    std::vector<double> u{2.0}; // domain space vector

    test0nJac("PowTestThree", &PowTestThreeFunc<double >, &PowTestThreeFunc<CG<double> >, u);
}

TEST_F(CppADCGOperationTest, PowTestFour) {
    std::vector<double> u{-2}; // domain space vector

    test0nJac("PowTestFour", &PowTestFourFunc<double >, &PowTestFourFunc<CG<double> >, u);
}

TEST_F(CppADCGOperationTest, PowTestFive) {
    std::vector<double> u{-1}; // domain space vector

    test0nJac("PowTestFive", &PowTestFiveFunc<double >, &PowTestFiveFunc<CG<double> >, u);
}

TEST_F(CppADCGOperationTest, PowTestSix) {
    std::vector<double> u{1.5}; // domain space vector

    test0nJac("PowTestSix", &PowTestSixFunc<double >, &PowTestSixFunc<CG<double> >, u);
}
