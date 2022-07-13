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
#include "sub.hpp"

using namespace CppAD;
using namespace CppAD::cg;

TEST_F(CppADCGOperationTest, subOne) {
    std::vector<double> u{3, 2}; // independent variable vector

    test0nJac("SubOne", &OneFunc<double >, &OneFunc<CG<double> >, u);
}

TEST_F(CppADCGOperationTest, subTwo) {
    std::vector<double> u{.5}; // independent variable vector

    test0nJac("SubTwo", &TwoFunc<double >, &TwoFunc<CG<double> >, u, 1e-10, 1e-10);
}

TEST_F(CppADCGOperationTest, subThree) {
    std::vector<double> u{1}; // independent variable vector

    test0nJac("SubThree", &ThreeFunc<double >, &ThreeFunc<CG<double> >, u, 1e-10, 1e-10);
}

TEST_F(CppADCGOperationTest, subFour) {
    std::vector<double> u{1}; // independent variable vector

    test0nJac("SubFour", &FourFunc<double >, &FourFunc<CG<double> >, u, 1e-10, 1e-10);
}

TEST_F(CppADCGOperationTest, subFive) {
    std::vector<double> u{1}; // independent variable vector

    test0nJac("SubFive", &FiveFunc<double >, &FiveFunc<CG<double> >, u, 1e-10, 1e-10);
}
