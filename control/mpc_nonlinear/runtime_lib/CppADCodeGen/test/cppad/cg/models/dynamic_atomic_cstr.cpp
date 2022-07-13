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
#include "CppADCGDynamicAtomicTest.hpp"
#include "cstr.hpp"

namespace CppAD {
namespace cg {

class CppADCGDynamicAtomicCstrTest : public CppADCGDynamicAtomicTest {
protected:
    static const size_t n;
    static const size_t m;
    CppAD::vector<Base> x;
    CppAD::vector<Base> xNorm;
    CppAD::vector<Base> eqNorm;
    std::vector<std::set<size_t> > jacInner, hessInner;
    std::vector<std::set<size_t> > jacOuter, hessOuter;
public:

    inline CppADCGDynamicAtomicCstrTest(bool verbose = false, bool printValues = false) :
        CppADCGDynamicAtomicTest("cstr_atomic", verbose, printValues),
        x(n),
        xNorm(n),
        eqNorm(m),
        jacInner(m), hessInner(6),
        jacOuter(m - 1), hessOuter(6) {
        this->verbose_ = false;

        using namespace std;
        using CppAD::vector;

        for (size_t i = 0; i < n; i++)
            x[i] = 1.0;

        xNorm[0] = 0.3; // h
        xNorm[1] = 7.82e3; // Ca
        xNorm[2] = 304.65; // Tr
        xNorm[3] = 301.15; // Tj

        xNorm[4] = 2.3333e-04; // u1
        xNorm[5] = 6.6667e-05; // u2

        xNorm[6] = 6.2e14; // 
        xNorm[7] = 10080; //
        xNorm[8] = 2e3; //
        xNorm[9] = 10e3; //
        xNorm[10] = 1e-11; //
        xNorm[11] = 6.6667e-05; //
        xNorm[12] = 294.15; //
        xNorm[13] = 294.15; //
        xNorm[14] = 1000; //
        xNorm[15] = 4184; //Cp
        xNorm[16] = -33488; //deltaH
        xNorm[17] = 299.15; // Tj0
        xNorm[18] = 302.65; //   Tj2
        xNorm[19] = 7e5; // cwallj
        xNorm[20] = 1203; // csteam
        xNorm[21] = 3.22; //dsteam
        xNorm[22] = 950.0; //Ug
        xNorm[23] = 0.48649427192323; //vc6in
        xNorm[24] = 1000; //rhoj
        xNorm[25] = 4184; //Cpj
        xNorm[26] = 0.014; //Vj
        xNorm[27] = 1e-7; //cwallr

        eqNorm[0] = 0.3;
        eqNorm[1] = 7.82e3;
        eqNorm[2] = 304.65;
        eqNorm[3] = 301.15;

        /**
         * Elements for the custom Jacobian/Hessian tests
         */
        jacInner[0].insert(0);
        jacInner[0].insert(1);
        jacInner[0].insert(2);
        jacInner[0].insert(3);
        jacInner[0].insert(5);
        jacInner[1].insert(0);
        jacInner[1].insert(1);
        jacInner[1].insert(2);
        jacInner[1].insert(3);
        jacInner[1].insert(5);
        jacInner[2].insert(0);
        jacInner[2].insert(1);
        jacInner[2].insert(2);
        jacInner[2].insert(3);
        jacInner[3].insert(0);
        jacInner[3].insert(2);
        jacInner[3].insert(3);
        jacInner[3].insert(4);

        hessInner[0].insert(0); // lower left side (with 1 exception)
        hessInner[0].insert(1);
        hessInner[0].insert(2);
        hessInner[0].insert(3);
        hessInner[0].insert(5);
        hessInner[1].insert(1);
        hessInner[1].insert(2);
        hessInner[1].insert(3);
        hessInner[1].insert(5);
        hessInner[2].insert(2);
        hessInner[2].insert(3);
        hessInner[2].insert(5);
        hessInner[3].insert(3);
        hessInner[4].insert(3); // flipped

        jacOuter[0] = jacInner[0];
        jacOuter[1] = jacInner[1];
        jacOuter[2] = jacInner[2];
        jacOuter[2].insert(jacInner[3].begin(), jacInner[3].end());

        hessOuter = hessInner; // only lower left side
        hessOuter[4].erase(3);
        hessOuter[3].insert(4);
    }

    virtual std::vector<ADCGD> model(const std::vector<ADCGD>& u) {
        return CstrFunc<CGD>(u);
    }

};

const size_t CppADCGDynamicAtomicCstrTest::n = 28;
const size_t CppADCGDynamicAtomicCstrTest::m = 4;

TEST_F(CppADCGDynamicAtomicCstrTest, ADFunAtomicLib) {
    this->testADFunAtomicLibSimple(x, xNorm, eqNorm, 1e-14, 1e-13);

    this->testAtomicSparsities(x);
}

TEST_F(CppADCGDynamicAtomicCstrTest, AtomicLibAtomicLib) {
    this->testAtomicLibAtomicLib(x, xNorm, eqNorm, 1e-14, 1e-13);
}

TEST_F(CppADCGDynamicAtomicCstrTest, AtomicLibModelBridge) {
    this->testAtomicLibModelBridge(x, xNorm, eqNorm, 1e-14, 1e-13);
}

TEST_F(CppADCGDynamicAtomicCstrTest, AtomicLibModelBridgeCustomRev2) {
    this->testAtomicLibModelBridgeCustom(x, xNorm, eqNorm,
                                         jacInner, hessInner,
                                         jacOuter, hessOuter,
                                         true,
                                         1e-14, 1e-13);
}

TEST_F(CppADCGDynamicAtomicCstrTest, AtomicLibModelBridgeCustomDirect) {
    this->testAtomicLibModelBridgeCustom(x, xNorm, eqNorm,
                                         jacInner, hessInner,
                                         jacOuter, hessOuter,
                                         false,
                                         1e-14, 1e-13);
}

} // END cg namespace
} // END CppAD namespace
