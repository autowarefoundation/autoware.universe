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
 * @test Model with 2 equations which share indexed temporary variables, but 
 *       also across iterations
 */
std::vector<ADCGD> modelCrossIteration1(const std::vector<ADCGD>& x, size_t repeat) {
    size_t m = 2;
    size_t n = 2;
    size_t m2 = repeat * m;

    // dependent variable vector 
    std::vector<ADCGD> y(m2);

    ADCGD tmp1 = cos(x[0]);
    ADCGD aux, aux1;
    for (size_t i = 0; i < repeat; i++) {
        if (i == 0) {
            y[i * m] = 1; // dep 0
            aux = x[i * n] * 2.5;
        } else {

            y[i * m] = aux; // dep 2 4 6...
        }

        aux1 = x[i * n] * 2.5;
        y[i * m + 1] = tmp1 * aux; // dep 1 3 5 ...
        aux = aux1;
    }

    return y;
}

TEST_F(CppADCGPatternTest, modelCrossIteration1) {
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

    setModel(modelCrossIteration1);

    testPatternDetection(m, n, repeat, loops);

    testLibCreation("modelCrossIteration1", m, n, repeat);
}

/**
 * @test Model with 2 equations which share indexed temporary variables, but 
 *       also across iterations
 */
std::vector<ADCGD> modelCrossIteration2(const std::vector<ADCGD>& x, size_t repeat) {
    size_t m = 2;
    size_t n = 2;
    size_t m2 = repeat * m;

    // dependent variable vector 
    std::vector<ADCGD> y(m2);

    ADCGD tmp1 = cos(x[0]);
    ADCGD aux, aux1;
    for (size_t i = 0; i < repeat; i++) {
        if (i == 0) {
            y[i * m] = 1; // dep 0
            aux = x[i * n] * 2.5;
        } else {
            y[i * m] = x[i * n] * tmp1 * aux; // dep 2 4 6...
        }

        aux1 = x[i * n] * 2.5;
        y[i * m + 1] = x[i * n + 1] * tmp1 * (aux1 - aux); // dep 1 3 5 ...
        aux = aux1;
    }

    return y;
}

TEST_F(CppADCGPatternTest, modelCrossIteration2) {
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

    //customJacSparsity_.resize(m * repeat);
    //customJacSparsity_[1].insert(0);
    //customHessSparsity_.resize(n * repeat);
    //customHessSparsity_[0].insert(0);
    setModel(modelCrossIteration2);

    testPatternDetection(m, n, repeat, loops);

    testLibCreation("modelCrossIteration2", m, n, repeat);
}
