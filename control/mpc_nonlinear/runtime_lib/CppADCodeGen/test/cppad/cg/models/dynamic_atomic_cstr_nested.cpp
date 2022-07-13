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
#include "cstr.hpp"

namespace CppAD {
namespace cg {

/**
 * Consider three CSTR models with the same parameters. 
 * The second CSTR has different states than the first CSTR.
 * The third CSTR has different states and controls than the previous two
 * CSTRs.
 * 
 * Inner model independent variables: [states, control, parameters]
 * Outer model independent variables: [states1, control12, parameters, states2, states3, control3]
 */
class CppADCGDynamicAtomicCstrNestedTest : public CppADCGDynamicAtomicNestedTest {
protected:
    /// total number of independent variables (states+controls+parameters)
    static const size_t n;
    /// number of states
    static const size_t ns;
    ///number of controls
    static const size_t nm;
    /// number of equations
    static const size_t m;
    CppAD::vector<Base> xOuter;
    CppAD::vector<Base> xInner;
    CppAD::vector<Base> xNorm;
    CppAD::vector<Base> eqNorm;
    std::vector<std::set<size_t> > jacInner, hessInner;
    std::vector<std::set<size_t> > jacOuter, hessOuter;
public:

    inline CppADCGDynamicAtomicCstrNestedTest(bool verbose = false, bool printValues = false) :
        CppADCGDynamicAtomicNestedTest("cstr_atomic", verbose, printValues),
        xOuter(n + ns + ns + nm),
        xInner(n),
        xNorm(n),
        eqNorm(m),
        jacInner(m), hessInner(n),
        jacOuter(3 * m), hessOuter(xOuter.size()) {
        this->verbose_ = false;

        using namespace std;
        using CppAD::vector;

        for (size_t i = 0; i < xInner.size(); i++)
            xInner[i] = 1.0;

        for (size_t i = 0; i < xOuter.size(); i++)
            xOuter[i] = 1.0;

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
         * Elements for the custom inner Jacobian/Hessian tests
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

        /**
         * Outer
         */
        std::copy(jacInner.begin(), jacInner.end(), jacOuter.begin());
        for (size_t i = 0; i < m; i++) {
            std::set<size_t>::const_iterator it;
            for (it = jacOuter[i].begin(); it != jacOuter[i].end(); ++it) {
                size_t j = *it;
                if (j < ns) {
                    jacOuter[i + m].insert(n + j); //second CSTR
                }
                jacOuter[i + 2 * m].insert(n + ns + j); //third CSTR
            }
        }

        std::copy(hessInner.begin(), hessInner.end(), hessOuter.begin());
        // only lower left side
        hessOuter[4].erase(3);
        hessOuter[3].insert(4);
        for (size_t j = 0; j < n; j++) {
            std::set<size_t>::const_iterator it;
            for (it = hessOuter[j].begin(); it != hessOuter[j].end(); ++it) {
                size_t jj = *it;
                if (j < ns && jj < ns) {
                    hessOuter[n + j].insert(n + jj); //second CSTR
                }
                hessOuter[n + ns + j].insert(n + ns + jj); //third CSTR
            }
        }

        for (size_t j = ns; j < ns + nm; j++) { // controls of the first & second CSTR
            std::set<size_t>::const_iterator it;
            std::set<size_t> hessOuterj = hessOuter[j]; //copy
            for (it = hessOuterj.begin(); it != hessOuterj.end(); ++it) {
                size_t jj = *it;
                if (jj < ns) {
                    hessOuter[j].insert(n + jj); //states of second CSTR 
                }
            }
        }
    }

    virtual std::vector<ADCGD> modelInner(const std::vector<ADCGD>& u) {
        return CstrFunc<CGD>(u);
    }

    virtual std::vector<ADCGD> modelOuter(const std::vector<ADCGD>& xOuter,
                                          atomic_base<CGD>& atomicInnerModel,
                                          size_t xInnerSize,
                                          size_t yInnerSize) {
        assert(xInnerSize == n);
        assert(yInnerSize == m);

        std::vector<ADCGD> yOuter(m * 3);
        std::vector<ADCGD> xInner(n), yInner(m);

        /**
         * first CSTR
         */
        std::copy(xOuter.begin(), xOuter.begin() + xInnerSize, xInner.begin()); //copy [states1, controls12, parameters]
        atomicInnerModel(xInner, yInner);
        std::copy(yInner.begin(), yInner.end(), yOuter.begin());

        /**
         * second CSTR
         */
        std::copy(xOuter.begin() + n, xOuter.begin() + n + ns, xInner.begin()); //copy [states2]
        atomicInnerModel(xInner, yInner);
        std::copy(yInner.begin(), yInner.end(), yOuter.begin() + m);

        /**
         * third CSTR
         */
        std::copy(xOuter.begin() + n + ns, xOuter.end(), xInner.begin()); //copy [states3, controls3]
        atomicInnerModel(xInner, yInner);
        std::copy(yInner.begin(), yInner.end(), yOuter.begin() + 2 * m);

        return yOuter;
    }

};

const size_t CppADCGDynamicAtomicCstrNestedTest::n = 28;
const size_t CppADCGDynamicAtomicCstrNestedTest::ns = 4;
const size_t CppADCGDynamicAtomicCstrNestedTest::nm = 2;
const size_t CppADCGDynamicAtomicCstrNestedTest::m = CppADCGDynamicAtomicCstrNestedTest::ns;

TEST_F(CppADCGDynamicAtomicCstrNestedTest, AtomicLibAtomicLib) {
    this->testAtomicLibAtomicLib(xOuter, xInner, xNorm, eqNorm, 1e-14, 1e-13);
}

TEST_F(CppADCGDynamicAtomicCstrNestedTest, AtomicLibModelBridge) {
    this->testAtomicLibModelBridge(xOuter, xInner, xNorm, eqNorm, 1e-14, 1e-13);
}

TEST_F(CppADCGDynamicAtomicCstrNestedTest, AtomicLibModelBridgeCustomRev2) {
    this->testAtomicLibModelBridgeCustom(xOuter, xInner, xNorm, eqNorm,
                                         jacInner, hessInner,
                                         jacOuter, hessOuter,
                                         true,
                                         1e-14, 1e-13);
}

TEST_F(CppADCGDynamicAtomicCstrNestedTest, AtomicLibModelBridgeCustomDirect) {
    this->testAtomicLibModelBridgeCustom(xOuter, xInner, xNorm, eqNorm,
                                         jacInner, hessInner,
                                         jacOuter, hessOuter,
                                         false,
                                         1e-14, 1e-13);
}

} // END cg namespace
} // END CppAD namespace