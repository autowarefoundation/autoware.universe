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
#include "CppADCGPatternTest.hpp"

using Base = double;
using CGD = CppAD::cg::CG<Base>;
using ADCGD = CppAD::AD<CGD>;

using namespace CppAD;
using namespace CppAD::cg;

/**
 * @test Model with 2 equations (1 is not defined for one of the iterations)
 *        which share only a non-indexed temporary variable
 */
std::vector<ADCGD> modelMissingEq1(const std::vector<ADCGD>& x, size_t repeat) {
    size_t m = 2;
    size_t n = 2;
    size_t m2 = repeat * m;

    // dependent variable vector 
    std::vector<ADCGD> y(m2);

    ADCGD tmp1 = cos(x[0]);
    for (size_t i = 0; i < repeat; i++) {
        if (i == 0) {
            y[i * m] = 1; // dep 0
        } else {
            y[i * m] = x[i * n] * tmp1 * 3; // dep 2 4 6...
        }

        y[i * m + 1] = x[i * n + 1] * tmp1 * (x[i * n + 1] - x[i * n]); // dep 1 3 5 ...
    }

    return y;
}

TEST_F(CppADCGPatternTest, modelMissingEq1) {
    size_t m = 2;
    size_t n = 2;
    size_t repeat = 6;

    std::vector<std::vector<std::set<size_t> > > loops(1);
    loops[0].resize(2);
    for (size_t i = 0; i < repeat; i++) {
        if (i != 0)
            loops[0][0].insert(i * m);
        loops[0][1].insert(i * m + 1);
    }

    setModel(modelMissingEq1);

    testPatternDetection(m, n, repeat, loops);

    testLibCreation("modelMissingEq1", m, n, repeat);
}

/**
 * @test Model with 2 equations (1 is not defined for one of the iterations)
 *        which share an indexed and a non-indexed temporary variables
 */
std::vector<ADCGD> modelMissingEq2(const std::vector<ADCGD>& x, size_t repeat) {
    size_t m = 2;
    size_t n = 2;
    size_t m2 = repeat * m;

    // dependent variable vector 
    std::vector<ADCGD> y(m2);

    ADCGD tmp1 = cos(x[0]);
    for (size_t i = 0; i < repeat; i++) {
        ADCGD aux = x[i * n] * 2.5;
        if (i == 0) {
            y[i * m] = 1; // dep 0
        } else {
            y[i * m] = aux * tmp1 * 3; // dep 2 4 6...
        }

        //y[i * m + 1] = x[i * n + 1] * tmp1 * (x[i * n + 1] - aux); // dep 1 3 5 ...
        y[i * m + 1] = tmp1 * (x[i * n + 1] - aux); // dep 1 3 5 ...
    }

    return y;
}

TEST_F(CppADCGPatternTest, modelMissingEq2) {
    size_t m = 2;
    size_t n = 2;
    size_t repeat = 6;

    std::vector<std::vector<std::set<size_t> > > loops(1);
    loops[0].resize(2);
    for (size_t i = 0; i < repeat; i++) {
        if (i != 0)
            loops[0][0].insert(i * m);
        loops[0][1].insert(i * m + 1);
    }

    setModel(modelMissingEq2);

    testPatternDetection(m, n, repeat, loops);

    testLibCreation("modelMissingEq2", m, n, repeat);
}
