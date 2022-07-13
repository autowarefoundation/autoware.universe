/* --------------------------------------------------------------------------
 *  CppADCodeGen: C++ Algorithmic Differentiation with Source Code Generation:
 *    Copyright (C) 2019 Joao Leal
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
#include "CppADCGDynamicTest.hpp"

namespace CppAD {
namespace cg {

class CppADCGDynamicTest1 : public CppADCGDynamicTest {
public:

    inline explicit CppADCGDynamicTest1() :
            CppADCGDynamicTest("dynamic", false, false) {
        _xTape = {1, 1, 1};
        _xRun = {1, 2, 1};
    }

    std::vector<ADCGD> model(const std::vector<ADCGD>& x) override {
        std::vector<ADCGD> y(2);

        y[0] = cos(x[0]);
        y[1] = x[1] * x[2] + sin(x[0]);

        return y;
    }

};

} // END cg namespace
} // END CppAD namespace

using namespace CppAD;
using namespace CppAD::cg;
using namespace std;

TEST_F(CppADCGDynamicTest1, ForwardZero1Assign) {
    _maxAssignPerFunc = 1;
    this->testForwardZero();
}

TEST_F(CppADCGDynamicTest1, ForwardZero) {
    _maxAssignPerFunc = 1000;
    this->testForwardZero();
}

TEST_F(CppADCGDynamicTest1, DenseJacobian) {
    this->testDenseJacobian();
}

TEST_F(CppADCGDynamicTest1, DenseHessian) {
    this->testDenseHessian();
}

TEST_F(CppADCGDynamicTest1, Jacobian) {
    this->testJacobian();
}

TEST_F(CppADCGDynamicTest1, Hessian) {
    this->testHessian();
}


namespace CppAD {
namespace cg {

class CppADCGDynamicTestCustomSparsity1 : public CppADCGDynamicTest1 {
public:

    inline explicit CppADCGDynamicTestCustomSparsity1() :
            CppADCGDynamicTest1() {
        // all elements except 1
        _jacRow = {0, 1, 1};
        _jacCol = {0, 0, 2};

        // all elements except 1
        _hessRow = {0, 2};
        _hessCol = {0, 1};
    }

};

} // END cg namespace
} // END CppAD namespace

TEST_F(CppADCGDynamicTestCustomSparsity1, Jacobian) {
    this->testJacobian();
}

TEST_F(CppADCGDynamicTestCustomSparsity1, Hessian) {
    this->testHessian();
}