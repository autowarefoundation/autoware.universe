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
#include "div.hpp"

using namespace CppAD;
using namespace CppAD::cg;

TEST_F(CppADCGOperationTest, DivTestOne) {
    // independent variable vector
    std::vector<double> u{2, 3};

    test0nJac("DivTestOne", &DivTestOneFunc<double >, &DivTestOneFunc<CG<double> >, u);
}

TEST_F(CppADCGOperationTest, DivTestTwo) {
    // independent variable vector
    std::vector<double> u{.5};

    test0nJac("DivTestTwo", &DivTestTwoFunc<double >, &DivTestTwoFunc<CG<double> >, u);
}

TEST_F(CppADCGOperationTest, DivTestThree) {
    // more testing of variable / variable case 
    std::vector<double> u{2, 3};

    test0nJac("DivTestThree", &DivTestThreeFunc<double >, &DivTestThreeFunc<CG<double> >, u);
}
