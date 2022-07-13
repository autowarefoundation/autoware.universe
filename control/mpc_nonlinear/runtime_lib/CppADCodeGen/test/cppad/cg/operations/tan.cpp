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
#include "tan.hpp"

using namespace CppAD;
using namespace CppAD::cg;

TEST_F(CppADCGOperationTest, tan) {
    double eps = 100. * std::numeric_limits<double>::epsilon();

    std::vector<double> u{.7}; // independent variable vector

    test0nJac("tan_first", &tanFirstFunc<double >, &tanFirstFunc<CG<double> >, u, eps, eps);

    test0nJac("tan_last", &tanLastFunc<double >, &tanLastFunc<CG<double> >, u, eps, eps);
}

TEST_F(CppADCGOperationTest, tanh) {
    double eps = 100. * std::numeric_limits<double>::epsilon();

    std::vector<double> u{.5}; // independent variable vector

    test0nJac("tanh_first", &tanhFirstFunc<double >, &tanhFirstFunc<CG<double> >, u, eps, eps);

    test0nJac("tanh_last", &tanhLastFunc<double >, &tanhLastFunc<CG<double> >, u, eps, eps);
}
