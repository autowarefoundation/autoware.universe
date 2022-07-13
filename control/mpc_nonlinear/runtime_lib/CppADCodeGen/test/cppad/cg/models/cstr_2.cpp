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
#include "../CppADCGDynamicTest.hpp"
#include "cstr.hpp"

namespace CppAD {
namespace cg {

class Cstr2DynamicTest : public CppADCGDynamicTest {
public:

    explicit Cstr2DynamicTest(bool verbose = false, bool printValues = false) :
        CppADCGDynamicTest("cstr2", verbose, printValues) {
        // independent variable vector
        _xNorm.resize(28 * 2);
        _xNorm[0] = 0.3; // h
        _xNorm[1] = 7.82e3; // Ca
        _xNorm[2] = 304.65; // Tr
        _xNorm[3] = 301.15; // Tj

        _xNorm[4] = 2.3333e-04; // u1
        _xNorm[5] = 6.6667e-05; // u2

        _xNorm[6] = 6.2e14; // 
        _xNorm[7] = 10080; //
        _xNorm[8] = 2e3; //
        _xNorm[9] = 10e3; //
        _xNorm[10] = 1e-11; //
        _xNorm[11] = 6.6667e-05; //
        _xNorm[12] = 294.15; //
        _xNorm[13] = 294.15; //
        _xNorm[14] = 1000; //
        _xNorm[15] = 4184; //Cp
        _xNorm[16] = -33488; //deltaH
        _xNorm[17] = 299.15; // Tj0
        _xNorm[18] = 302.65; //   Tj2
        _xNorm[19] = 7e5; // cwallj
        _xNorm[20] = 1203; // csteam
        _xNorm[21] = 3.22; //dsteam
        _xNorm[22] = 950.0; //Ug
        _xNorm[23] = 0.48649427192323; //vc6in
        _xNorm[24] = 1000; //rhoj
        _xNorm[25] = 4184; //Cpj
        _xNorm[26] = 0.014; //Vj
        _xNorm[27] = 1e-7; //cwallr


        _eqNorm.resize(4 * 2);
        _eqNorm[0] = 0.3;
        _eqNorm[1] = 7.82e3;
        _eqNorm[2] = 304.65;
        _eqNorm[3] = 301.15;

        size_t n = _xNorm.size()/2;
        std::copy(_xNorm.begin(), _xNorm.begin() + n, _xNorm.begin() + n);

        _xRun = std::vector<double>(_xNorm.size(), 1.0);
        _xTape = _xRun;

        size_t m = _eqNorm.size() / 2;
        std::copy(_eqNorm.begin(), _eqNorm.begin() + m, _eqNorm.begin() + m);

        _maxAssignPerFunc = 1000;
    }

    std::vector<ADCGD> model(const std::vector<ADCGD>& ind) override {
        std::vector<ADCGD> eqs(8);
        std::vector<ADCGD> ind1(28);
        std::copy(ind.begin(), ind.begin() + 28, ind1.begin());
        std::vector<ADCGD> sys1 = CstrFunc<CG<double> >(ind1);
        std::copy(sys1.begin(), sys1.end(), eqs.begin());

        std::vector<ADCGD> ind2(28);
        std::copy(ind.begin() + 28, ind.end(), ind2.begin());
        std::vector<ADCGD> sys2 = CstrFunc<CG<double> >(ind2);
        std::copy(sys2.begin(), sys2.end(), eqs.begin() + sys1.size());
        return eqs;
    }

};

} // END cg namespace
} // END CppAD namespace

using namespace CppAD;
using namespace CppAD::cg;

TEST_F(Cstr2DynamicTest, ForwardZero) {
    this->testForwardZero();
}

TEST_F(Cstr2DynamicTest, DenseJacobian) {
    this->testDenseJacobian();
}

TEST_F(Cstr2DynamicTest, DenseHessian) {
    this->testDenseHessian();
}

TEST_F(Cstr2DynamicTest, Jacobian) {
    this->testJacobian();
}

TEST_F(Cstr2DynamicTest, Hessian) {
    this->testHessian();
}
