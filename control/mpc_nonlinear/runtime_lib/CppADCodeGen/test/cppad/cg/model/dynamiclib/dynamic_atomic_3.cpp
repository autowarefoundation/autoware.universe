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
#include "CppADCGDynamicAtomicNestedTest.hpp"

namespace CppAD {
namespace cg {

/**
 * 
 */
class CppADCGDynamicAtomicSmallerNestedTest : public CppADCGDynamicAtomicNestedTest {
protected:
    /// number of equations
    static const size_t m;
    static const size_t n;
    CppAD::vector<Base> xOuter;
    CppAD::vector<Base> xInner;
    CppAD::vector<Base> xNorm;
    CppAD::vector<Base> eqNorm;
    std::vector<std::set<size_t> > jacInner, hessInner;
    std::vector<std::set<size_t> > jacOuter, hessOuter;
public:

    inline CppADCGDynamicAtomicSmallerNestedTest(bool verbose = false, bool printValues = false) :
        CppADCGDynamicAtomicNestedTest("simple_atomic", verbose, printValues),
        xOuter(n),
        xInner(n * 2),
        xNorm(n * 2),
        eqNorm(m),
        jacOuter(m), hessOuter(n) {
        this->verbose_ = false;

        using namespace std;
        using CppAD::vector;

        for (size_t j = 0; j < xInner.size(); j++) {
            xInner[j] = 1.0;
            xNorm[j] = 1.0;
        }

        for (size_t i = 0; i < eqNorm.size(); i++) {
            eqNorm[i] = 1.0;
        }

        for (size_t j = 0; j < xOuter.size(); j++)
            xOuter[j] = 1.0;

        jacOuter[0].insert(0);
        hessOuter[0].insert(0);
    }

    std::vector<ADCGD> modelInner(const std::vector<ADCGD>& u) override {
        std::vector<ADCGD> y(m);
        y[0] = u[0] * u[1];
        return y;
    }

    std::vector<ADCGD> modelOuter(const std::vector<ADCGD>& xOuter,
                                  atomic_base<CGD>& atomicInnerModel,
                                  size_t xInnerSize,
                                  size_t yInnerSize) override {
        assert(xInnerSize == 2 * n);
        assert(yInnerSize == m);

        std::vector<ADCGD> xInner(2 * n), yInner(m);
        xInner[0] = xOuter[0];
        xInner[1] = xOuter[0];

        atomicInnerModel(xInner, yInner);

        std::vector<ADCGD> yOuter(m);
        yOuter[0] = yInner[0];

        return yOuter;
    }

};

const size_t CppADCGDynamicAtomicSmallerNestedTest::n = 1;
const size_t CppADCGDynamicAtomicSmallerNestedTest::m = 1;

TEST_F(CppADCGDynamicAtomicSmallerNestedTest, AtomicLibAtomicLib) {
    this->testAtomicLibAtomicLib(xOuter, xInner, xNorm, eqNorm, 1e-14, 1e-13);
}

TEST_F(CppADCGDynamicAtomicSmallerNestedTest, AtomicLibModelBridge) {
    this->testAtomicLibModelBridge(xOuter, xInner, xNorm, eqNorm, 1e-14, 1e-13);
}

TEST_F(CppADCGDynamicAtomicSmallerNestedTest, AtomicLibModelBridgeCustomRev2) {
    this->testAtomicLibModelBridgeCustom(xOuter, xInner, xNorm, eqNorm,
                                         jacInner, hessInner,
                                         jacOuter, hessOuter,
                                         true,
                                         1e-14, 1e-13);
}

TEST_F(CppADCGDynamicAtomicSmallerNestedTest, AtomicLibModelBridgeCustomDirect) {
    this->testAtomicLibModelBridgeCustom(xOuter, xInner, xNorm, eqNorm,
                                         jacInner, hessInner,
                                         jacOuter, hessOuter,
                                         false,
                                         1e-14, 1e-13);
}

} // END cg namespace
} // END CppAD namespace