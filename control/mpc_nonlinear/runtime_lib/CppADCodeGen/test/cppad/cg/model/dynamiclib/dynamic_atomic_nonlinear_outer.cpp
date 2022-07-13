/* --------------------------------------------------------------------------
 *  CppADCodeGen: C++ Algorithmic Differentiation with Source Code Generation:
 *    Copyright (C) 2019 Joao Leal
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
#include "CppADCGDynamicAtomicTest.hpp"

namespace CppAD {
namespace cg {

/**
 * Linear inner model and an outer model which performs a nonlinear
 * operation with the result of the inner model
 */
class CppADCGDynamicAtomicModel3Test : public CppADCGDynamicAtomicTest {
protected:
    static const size_t n = 2;
    static const size_t m = 1;
public:

    inline CppADCGDynamicAtomicModel3Test(bool verbose = false,
                                          bool printValues = false) :
            CppADCGDynamicAtomicTest("dynamicAtomic3", verbose, printValues) {
        this->verbose_ = false;
    }

    std::vector<ADCGD> model(const std::vector<ADCGD>& x) override {
        std::vector<ADCGD> y(m);

        y[0] = x[0] * 2 + x[1] * 3;

        return y;
    }

    std::vector<ADCGD> modelOuter(const std::vector<ADCGD>& y) override {
        std::vector<ADCGD> z(m);

        z[0] = 2 * y[0] * y[0];

        return z;
    }

};

/**
 * Nonlinear inner model and an outer model which performs nonlinear
 * operations with the result of the inner model
 */
class CppADCGDynamicAtomicModel4Test : public CppADCGDynamicAtomicTest {
protected:
    static const size_t n = 3;
    static const size_t m = 4;
public:

    inline CppADCGDynamicAtomicModel4Test(bool verbose = false,
                                          bool printValues = false) :
        CppADCGDynamicAtomicTest("dynamicAtomic4", verbose, printValues) {
        this->verbose_ = false;
    }

    std::vector<ADCGD> model(const std::vector<ADCGD>& x) override {
        std::vector<ADCGD> y(m);

        y[0] = cos(x[0]);
        y[1] = x[1] * x[2] + sin(x[0]);
        y[2] = x[2] * x[2] + sin(x[1]);
        y[3] = x[0] / x[2] + x[1] * x[2] + 5.0;

        return y;
    }

    std::vector<ADCGD> modelOuter(const std::vector<ADCGD>& y) override {
        std::vector<ADCGD> z(m - 1);

        z[0] = 2 * y[0];
        z[1] = 2 * y[1] * y[0];
        z[2] = 2 * y[2] + y[3];

        return z;
    }

};

} // END cg namespace
} // END CppAD namespace

using namespace CppAD;
using namespace CppAD::cg;
using namespace std;


/**
 * @test linear inner model and an outer model which performs nonlinear
 *       operations with the result of the inner model
 */
TEST_F(CppADCGDynamicAtomicModel3Test, DynamicForRevLinearNonlinear) {
    using namespace std;
    using CppAD::vector;

    vector<Base> x(n);
    for (size_t j = 0; j < n; j++)
        x[j] = j + 2;

    // simple wrap (no outer model used)
    this->testADFunAtomicLibSimple(x); // one compiled model used as an atomic function by an ADFun

    this->testAtomicSparsities(x);

    // use outer model
    this->testADFunAtomicLib(x); // 1 compiled inner model used by CppAD

    this->testAtomicLibAtomicLib(x); // 2 models in 2 dynamic libraries
}

/**
 * @test nonlinear inner model and an outer model which performs linear
 *       operations with the result of the inner model
 */
TEST_F(CppADCGDynamicAtomicModel4Test, DynamicForRevNonlinearNonlinear) {
    using namespace std;
    using CppAD::vector;

    vector<Base> x(n);
    for (size_t j = 0; j < n; j++)
        x[j] = j + 2;

    // simple wrap (no outer model used)
    this->testADFunAtomicLibSimple(x); // one compiled model used as an atomic function by an ADFun

    this->testAtomicSparsities(x);

    // use outer model
    this->testADFunAtomicLib(x); // 1 compiled inner model used by CppAD

    this->testAtomicLibAtomicLib(x); // 2 models in 2 dynamic libraries
}
