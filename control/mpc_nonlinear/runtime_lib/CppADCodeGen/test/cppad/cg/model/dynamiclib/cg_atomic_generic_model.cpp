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
#include <utility>

#include "CGAtomicGenericModelTest.hpp"

using namespace std;
using namespace CppAD::cg;

namespace CppAD {
namespace cg {

/**
 * Nonlinear inner model with a single equation and an outer model which
 * performs linear operations with the result of the inner model
 */
class SingleVarAtomicGenericModelTest : public CGAtomicGenericModelTest {
protected:
    static const size_t n = 1;
    static const size_t m = 1;
public:

    inline SingleVarAtomicGenericModelTest() :
            CGAtomicGenericModelTest("singleVarAtomic", n) {
    }

    std::vector<ADCGD> model(const std::vector<ADCGD> &x) override {
        return {x[0] * x[0]};
    }

};


} // END cg namespace
} // END CppAD namespace


/**
 * @test
 */
TEST_F(SingleVarAtomicGenericModelTest, TestForwardZero) { // NOLINT(cert-err58-cpp)
    this->testCGAtomicGenericModelForwardZero();
}

TEST_F(SingleVarAtomicGenericModelTest, TestReverseOne) { // NOLINT(cert-err58-cpp)
    this->testCGAtomicGenericModelReverseOne();
}

TEST_F(SingleVarAtomicGenericModelTest, TestReverseTwo) { // NOLINT(cert-err58-cpp)
    this->testCGAtomicGenericModelReverseTwo();
}

TEST_F(SingleVarAtomicGenericModelTest, TestJacobian) { // NOLINT(cert-err58-cpp)
    this->testCGAtomicGenericModelJacobian();
}

TEST_F(SingleVarAtomicGenericModelTest, TestHessian) { // NOLINT(cert-err58-cpp)
    this->testCGAtomicGenericModelHessian();
}


namespace CppAD {
namespace cg {

/**
 * Nonlinear inner model with a single equation and an outer model which
 * performs linear operations with the result of the inner model
 */
class SingleVarAtomicGenericModelTest2 : public CGAtomicGenericModelTest {
protected:
    static const size_t n = 1;
    static const size_t m = 1;
public:

    inline SingleVarAtomicGenericModelTest2() :
            CGAtomicGenericModelTest("dynamicAtomic1", n) {}

    std::vector<ADCGD> model(const std::vector<ADCGD> &x) override {
        return {1.0 / x[0]};
    }

};

} // END cg namespace
} // END CppAD namespace

TEST_F(SingleVarAtomicGenericModelTest2, TestForwardZero) { // NOLINT(cert-err58-cpp)
    this->testCGAtomicGenericModelForwardZero();
}

TEST_F(SingleVarAtomicGenericModelTest2, TestReverseOne) { // NOLINT(cert-err58-cpp)
    this->testCGAtomicGenericModelReverseOne();
}

TEST_F(SingleVarAtomicGenericModelTest2, TestReverseTwo) { // NOLINT(cert-err58-cpp)
    this->testCGAtomicGenericModelReverseTwo();
}

TEST_F(SingleVarAtomicGenericModelTest2, TestJacobian) { // NOLINT(cert-err58-cpp)
    this->testCGAtomicGenericModelJacobian();
}

TEST_F(SingleVarAtomicGenericModelTest2, TestHessian) { // NOLINT(cert-err58-cpp)
    this->testCGAtomicGenericModelHessian();
}

namespace CppAD {
namespace cg {

/**
 * Multivariable nonlinear inner model and a linear outer model
 */
class MultiVarAtomicGenericModelTest : public CGAtomicGenericModelTest {
protected:
    static const size_t n = 2;
    static const size_t m = 1;
public:

    explicit MultiVarAtomicGenericModelTest(std::vector<std::set<size_t> > jacSpar = {},
                                            std::vector<std::set<size_t> > hessSpar = {}) :
            CGAtomicGenericModelTest("dynamicAtomic3", n, std::move(jacSpar), std::move(hessSpar)) {
    }

    std::vector<ADCGD> model(const std::vector<ADCGD> &x) override {
        return {x[0] * x[0] + x[0] * x[1] + x[1] * x[1]};
    }
};

} // END cg namespace
} // END CppAD namespace


TEST_F(MultiVarAtomicGenericModelTest, TestForwardZero) { // NOLINT(cert-err58-cpp)
    this->testCGAtomicGenericModelForwardZero();
}

TEST_F(MultiVarAtomicGenericModelTest, TestReverseOne) { // NOLINT(cert-err58-cpp)
    this->testCGAtomicGenericModelReverseOne();
}

TEST_F(MultiVarAtomicGenericModelTest, TestReverseTwo) { // NOLINT(cert-err58-cpp)
    this->testCGAtomicGenericModelReverseTwo();
}

TEST_F(MultiVarAtomicGenericModelTest, TestJacobian) { // NOLINT(cert-err58-cpp)
    this->testCGAtomicGenericModelJacobian();
}

TEST_F(MultiVarAtomicGenericModelTest, TestHessian) { // NOLINT(cert-err58-cpp)
    this->testCGAtomicGenericModelHessian();
}

#if 0
// https://github.com/joaoleal/CppADCodeGen/issues/31
namespace CppAD {
namespace cg {

class MultiVarAtomicGenericModelLowerTest : public MultiVarAtomicGenericModelTest {
public:
    MultiVarAtomicGenericModelLowerTest() : MultiVarAtomicGenericModelTest({}, {{},
                                                                                {0, 1}}) {}
};

} // END cg namespace
} // END CppAD namespace

TEST_F(MultiVarAtomicGenericModelLowerTest, TestHessian) { // NOLINT(cert-err58-cpp)
    this->testCGAtomicGenericModelHessian();
}

namespace CppAD {
namespace cg {

class MultiVarAtomicGenericModelUpperTest : public MultiVarAtomicGenericModelTest {
public:
    MultiVarAtomicGenericModelUpperTest() : MultiVarAtomicGenericModelTest({}, {{0, 1},
                                                                                {}}) {}
};

} // END cg namespace
} // END CppAD namespace

TEST_F(MultiVarAtomicGenericModelUpperTest, TestHessian) { // NOLINT(cert-err58-cpp)
    this->testCGAtomicGenericModelHessian();
}

#endif

namespace CppAD {
namespace cg {


/**
 * Nonlinear inner model and an outer model which performs linear
 * operations with the result of the inner model
 */
class MultiVarAtomicGenericModelTest2 : public CGAtomicGenericModelTest {
protected:
    static const size_t n = 3;
    static const size_t m = 4;
public:

    inline MultiVarAtomicGenericModelTest2() :
            CGAtomicGenericModelTest("dynamicAtomic2", n) {
    }

    std::vector<ADCGD> model(const std::vector<ADCGD> &x) override {
        std::vector<ADCGD> y(m);

        y[0] = cos(x[0]);
        y[1] = x[1] * x[2] + sin(x[0]);
        y[2] = x[2] * x[2] + sin(x[1]);
        y[3] = x[0] / x[2] + x[1] * x[2] + 5.0;

        return y;
    }

};

} // END cg namespace
} // END CppAD namespace


TEST_F(MultiVarAtomicGenericModelTest2, TestForwardZero) { // NOLINT(cert-err58-cpp)
    this->testCGAtomicGenericModelForwardZero();
}

TEST_F(MultiVarAtomicGenericModelTest2, TestReverseOne) { // NOLINT(cert-err58-cpp)
    this->testCGAtomicGenericModelReverseOne();
}

TEST_F(MultiVarAtomicGenericModelTest2, TestReverseTwo) { // NOLINT(cert-err58-cpp)
    this->testCGAtomicGenericModelReverseTwo();
}

TEST_F(MultiVarAtomicGenericModelTest2, TestJacobian) { // NOLINT(cert-err58-cpp)
    this->testCGAtomicGenericModelJacobian();
}

TEST_F(MultiVarAtomicGenericModelTest2, TestHessian) { // NOLINT(cert-err58-cpp)
    this->testCGAtomicGenericModelHessian();
}
