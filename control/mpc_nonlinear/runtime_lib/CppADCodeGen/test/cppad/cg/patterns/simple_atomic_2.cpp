/* --------------------------------------------------------------------------
 *  CppADCodeGen: C++ Algorithmic Differentiation with Source Code Generation:
 *    Copyright (C) 2015 Ciengis
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

using namespace CppAD;
using namespace CppAD::cg;

using Base = double;
using CGD = CppAD::cg::CG<Base>;
using ADCGD = CppAD::AD<CGD>;

/**
 * atomic function
 */
std::vector<ADCGD> modelAtomic(const std::vector<ADCGD>& x, size_t repeat, atomic_base<CGD>& atomic) {
    size_t m = 3;
    size_t n = 4;
    size_t m2 = repeat * m;

    // dependent variable vector 
    std::vector<ADCGD> y(m2), ax(4), ay(2);

    for (size_t i = 0; i < repeat; i++) {
        y[i * m] = cos(x[i * n]);

        ax[0] = x[i * n];
        ax[1] = x[i < repeat / 2 ? i * n + 1 : (repeat / 2 + 1) * n + 1];
        ax[2] = x[i < repeat / 2 ? i * n + 2 : (repeat / 2 + 1) * n + 2];
        ax[3] = x[i < repeat / 2 ? i * n + 3 : (repeat / 2 + 1) * n + 3];
        atomic(ax, ay);
        y[i * m + 1] = ay[0];
        y[i * m + 2] = ay[1];
    }

    return y;
}

void atomicFunction(const std::vector<AD<double> >& x,
                    std::vector<AD<double> >& y) {
    y[0] = 1 * x[0] * x[0] + x[2] * x[3];
    y[1] = 2 * x[0] * x[1];
}

/**
 * @test Test the correct generation of source for the assignment of arrays 
 * using "for" loops with more than one index
 */
TEST_F(CppADCGPatternTest, SimpleAtomic2) {
    using namespace CppAD;

    size_t m = 3;
    size_t n = 4;
    size_t repeat = 6;

    // create atomic function
    std::vector<AD<double> > y(2), x(n);
    checkpoint<double> atomicfun("atomicFunc2", atomicFunction, x, y);
    CGAtomicFun<double> cgatomicfun(atomicfun, x, true);
    PatternTestModelWithAtom<CGD> model(modelAtomic, cgatomicfun);
    setModel(model);
    this->atoms_.push_back(&atomicfun);

    testPatternDetection(m, n, repeat);
    testLibCreation("modelSimpleAtomic2", m, n, repeat);
}
