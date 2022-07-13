#ifndef CPPAD_CG_TEST_THREAD_POOL_TEST_INCLUDED
#define CPPAD_CG_TEST_THREAD_POOL_TEST_INCLUDED
/* --------------------------------------------------------------------------
 *  CppADCodeGen: C++ Algorithmic Differentiation with Source Code Generation:
 *    Copyright (C) 2019 Joao Leal
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
#include "CppADCGDynamicTest.hpp"

namespace CppAD {
namespace cg {

class ThreadPoolTest : public CppADCGDynamicTest {
public:
    using CGD = CG<double>;
    using ADCG = AD<CGD>;
public:
    explicit ThreadPoolTest(MultiThreadingType threadType,
                            bool verbose = true) :
            CppADCGDynamicTest("pool", verbose, false) {
        this->_multithread = threadType;

        this->_reverseOne = true;
        this->_reverseTwo = true;
        this->_denseJacobian = false;
        this->_denseHessian = false;

        this->_maxAssignPerFunc = 1000;

        // independent variables
        _xTape.resize(9);
        for (auto& ui : _xTape)
            ui = 1;

        _xRun.resize(_xTape.size());
        for (auto& xi : _xRun)
            xi = 1.5;
    }

    std::vector<ADCGD> model(const std::vector<ADCGD>& x) override {
        std::vector<ADCGD> y(6);

        for (size_t i = 0; i < 3; ++i) {
            size_t i0 = i * 2;
            size_t j0 = i * 3;

            y[i0] = cos(x[j0]);
            y[i0 + 1] = x[j0 + 1] * x[j0 + 2] + sin(x[j0]);
        }

        return y;
    }

};

} // END cg namespace
} // END CppAD namespace

#endif
