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
#include "cosh.hpp"

using namespace CppAD;
using namespace CppAD::cg;

TEST_F(CppADCGOperationTest, cosh) {
    // independent variable vector
    std::vector<double> u{1};

    test0nJac("cosh", &CoshFunc<double >, &CoshFunc<CG<double> >, u, 1e-10, 1e-10);
}
