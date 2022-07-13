/* --------------------------------------------------------------------------
 *  CppADCodeGen: C++ Algorithmic Differentiation with Source Code Generation:
 *    Copyright (C) 2013 Ciengis
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

#include <cppad/cg/cppadcg.hpp>
#include <gtest/gtest.h>

#include "CppADCGTest.hpp"

using namespace CppAD;
using namespace CppAD::cg;
using namespace std;

TEST_F(CppADCGTest, multMatrixMatrixSparsityTrans) {
    std::vector<std::set<size_t> > aT{
        {0},
        {1},
        {1},
        {0, 2}
    }; // a: 3 x 4
    std::vector<std::set<size_t> > b{
        {0},
        {},
        {0, 1},
        {1}
    }; // b: 4 x 2
    CppAD::vector<std::set<size_t> > rT(2); // r: 3 x 2 

    multMatrixMatrixSparsityTrans(aT, b, rT, 4, 2, 3);

    CppAD::vector<std::set<size_t> > rTExpected(2);
    rTExpected[0].insert(0);
    rTExpected[0].insert(1);
    rTExpected[1].insert(0);
    rTExpected[1].insert(1);
    rTExpected[1].insert(2);

    compareVectorSetValues(rT, rTExpected);
}

TEST_F(CppADCGTest, multMatrixMatrixSparsityTrans2) {
    size_t m = 2;
    size_t n = 7;
    size_t q = 33;
    std::vector<std::set<size_t> > sT{
        {29},
        {30}
    }; //size m

    std::vector<std::set<size_t> > jac{
        {0, 2},
        {1, 3}
    }; //size m

    CppAD::vector<std::set<size_t> > rT(n);

    multMatrixMatrixSparsityTrans(sT, jac, rT, m, n, q);

    CppAD::vector<std::set<size_t> > rTExpected(n);
    rTExpected[0].insert(29);
    rTExpected[1].insert(30);
    rTExpected[2].insert(29);
    rTExpected[3].insert(30);

    compareVectorSetValues(rT, rTExpected);
}

TEST_F(CppADCGTest, multMatrixMatrixSparsity) {
    size_t m = 4;
    size_t n = 3;
    size_t q = 2;
    std::vector<std::set<size_t> > a{
        {2},
        {1},
        {0, 1, 2},
        {0}
    }; // size m

    std::vector<std::set<size_t> > b{
        {0},
        {1},
        {0}
    }; // size n

    CppAD::vector<std::set<size_t> > r(m);

    multMatrixMatrixSparsity(a, b, r, m, n, q);

    CppAD::vector<std::set<size_t> > rExpected(m);
    rExpected[0].insert(0);
    rExpected[1].insert(1);
    rExpected[2].insert(0);
    rExpected[2].insert(1);
    rExpected[3].insert(0);

    compareVectorSetValues(r, rExpected);
}

TEST_F(CppADCGTest, multMatrixTransMatrixSparsity) {
    size_t m = 4;
    size_t n = 3;
    size_t q = 2;
    std::vector<std::set<size_t> > a{
        {2, 3},
        {1, 2},
        {0, 2}
    }; // size n

    std::vector<std::set<size_t> > b{
        {0},
        {1},
        {0}
    }; // size n

    CppAD::vector<std::set<size_t> > r(m);

    multMatrixTransMatrixSparsity(a, b, r, n, m, q);

    CppAD::vector<std::set<size_t> > rExpected(m);
    rExpected[0].insert(0);
    rExpected[1].insert(1);
    rExpected[2].insert(0);
    rExpected[2].insert(1);
    rExpected[3].insert(0);

    compareVectorSetValues(r, rExpected);
}
