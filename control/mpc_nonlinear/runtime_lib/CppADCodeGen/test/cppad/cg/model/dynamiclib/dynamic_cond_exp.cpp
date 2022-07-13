/* --------------------------------------------------------------------------
 *  CppADCodeGen: C++ Algorithmic Differentiation with Source Code Generation:
 *    Copyright (C) 2019 Joao Leal
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
#include "CppADCGDynamicTest.hpp"

namespace CppAD {
namespace cg {

class CppADCGDynamicTest1 : public CppADCGDynamicTest {
public:

    explicit CppADCGDynamicTest1(bool verbose = false, bool printValues = false) :
            CppADCGDynamicTest("dynamic_cond_exp", verbose, printValues) {
        // independent variables
        _xTape = {1, 1, 1, 1, 1, 1, 1};
        _xRun = {1, 2, 1, 1, 1, 1, 1};
        _maxAssignPerFunc = 10000;
    }

    virtual std::vector<ADCGD> model(const std::vector<ADCGD>& x) {
        std::vector<ADCGD> y(2);

        ADCGD zero = CGD(0);
        ADCGD one = CGD(1);

#if 0
        ADCGD v = sin(x[0]);
        ADCGD v4;
        for (int i = 0; i < 3; ++i) {
            ADCGD v2 = v + v;
            ADCGD v3 = v2 + v2;

            v4 = CondExpGt(v3, one, one, v3);
            v4 = CondExpLt(v4, zero, zero, v4);
            v4 *= 2.5;

            v = v4;
        }

        y[0] = v4;
        y[1] = cos(x[1]);
        y[2] = exp(y[2]);
#endif
        // auxiliary variables
        ADCGD v[15];

        v[0] = x[5] * x[3];
        v[1] = 1 - x[0];
        v[2] = (0.0180152833 * v[1]) / (0.0180152833 * v[1] + 0.04606844 * x[0]);
        v[3] = 1 - v[2];
        v[4] = 101325. * x[4];
        v[5] = v[1] * (-1435.264 / (log(v[4] / 100000.) / 2.30258509299405 - 4.6543) - -64.848) + x[0] * (-1432.526 / (log(v[4] / 100000.) / 2.30258509299405 - 4.92531) - -61.819);
        v[5] = -1432.526 / (log((v[4] / ((100000. * pow(10., 4.6543 - 1435.264 / (-64.848 + v[5]))) / (100000. * pow(10., 4.92531 - 1432.526 / (-61.819 + v[5]))) * v[1] + x[0])) / 100000.) / 2.30258509299405 - 4.92531) - -61.819;
        v[5] = -1432.526 / (log((v[4] / ((100000. * pow(10., 4.6543 - 1435.264 / (-64.848 + v[5]))) / (100000. * pow(10., 4.92531 - 1432.526 / (-61.819 + v[5]))) * v[1] + x[0])) / 100000.) / 2.30258509299405 - 4.92531) - -61.819;
        v[6] = 100000. * pow(10., 4.6543 - 1435.264 / (-64.848 + v[5]));
        v[5] = -1432.526 / (log((v[4] / (v[6] / (100000. * pow(10., 4.92531 - 1432.526 / (-61.819 + v[5]))) * v[1] + x[0])) / 100000.) / 2.30258509299405 - 4.92531) - -61.819;
        v[7] = 1 / (v[2] / (0.0180152833 * 1000. * (-13.851 + 0.64038 * v[5] + -0.00191 * v[5] * v[5] + 1.8211e-06 * v[5] * v[5] * v[5])) + v[3] / (0.04606844 * 1000. * 1.6288 / pow(0.27469, 1 + pow(1 - v[5] / 514., 0.23178))));
        v[8] = 0.0180152833 * v[1] + 0.04606844 * x[0];
        v[9] = v[3] * x[1] * v[8];
        v[10] = 1 / (v[2] / (0.0180152833 * 1000. * (-13.851 + 0.64038 * v[5] + -0.00191 * v[5] * v[5] + 1.8211e-06 * v[5] * v[5] * v[5])) + v[3] / (0.04606844 * 1000. * 1.6288 / pow(0.27469, 1 + pow(1 - v[5] / 514., 0.23178))));
        v[10] = 0.001 * sqrt(2. * (9.80665 * ((v[2] * x[1] * v[8] + v[9]) / v[10]) / 3.14159265358979 + (101325. - v[4]) / v[10]));
        v[3] = v[3] * v[7] * v[10];
        v[6] = (v[6] * v[1]) / v[4];
        v[6] = CondExpGt(v[6], one, one, v[6]);
        v[6] = CondExpLt(v[6], zero, zero, v[6]);
        v[4] = 1 - v[6];
        v[8] = v[6] + v[4];
        v[4] = v[4] / v[8];
        v[11] = (1 - x[5]) * x[3];
        v[12] = x[6] - -273.15;
        v[13] = v[12] * v[12];
        v[14] = v[5] * v[5];
        v[8] = v[6] / v[8];
        v[6] = v[5] / 647.;
        v[6] = (0.0180152833 * v[11] * (15340.8633879213 * (v[12] - v[5]) + -58.0090794353481 * (v[12] * v[12] - v[5] * v[5]) + 0.150335317420922 * (v[12] * v[12] * v[12] - v[5] * v[5] * v[5]) + -0.000195889231450498 * (v[12] * v[12] * v[12] * v[12] - v[5] * v[5] * v[5] * v[5]) + 1.04023898419627e-07 * (v[13] * v[13] * v[12] - v[14] * v[14] * v[5])) + 0.04606844 * v[0] * (2227.98948694594 * (v[12] - v[5]) + -1.51546264644516 * (v[12] * v[12] - v[5] * v[5]) + -0.000219535687917079 * (v[12] * v[12] * v[12] - v[5] * v[5] * v[5]) + 1.10628881724669e-05 * (v[12] * v[12] * v[12] * v[12] - v[5] * v[5] * v[5] * v[5])) + x[2]) / (v[8] * 52053. * pow(1 - v[6], 0.3199 + -0.212 * v[6] + 0.25795 * v[6] * v[6]) + v[4] * 50430. * pow(1 - v[5] / 514., 0.4989) * exp((0.4475 * v[5]) / 514.));
        v[4] = v[0] - v[3] / 0.04606844 - v[4] * v[6];
        y[1] = v[11] - v[1] * (v[2] * v[7] * v[10] + v[3]) / (0.0180152833 * v[1] + 0.04606844 * x[0]) - v[8] * v[6] + v[4];
        y[0] = (x[1] * v[4] - v[9] / 0.04606844 * y[1]) / (x[1] * x[1]);

        return y;
    }

};

} // END cg namespace
} // END CppAD namespace

using namespace CppAD;
using namespace CppAD::cg;
using namespace std;

#if 0
TEST_F(CppADCGDynamicTest1, DynamicCondExpCustom) {
    // use a special object for source code generation
    using CGD = CG<double>;
    using ADCG = AD<CGD>;

    // independent variables
    std::vector<ADCG> u(7);
    u[0] = 1;
    u[1] = 1;
    u[2] = 1;
    u[3] = 1;
    u[4] = 1;
    u[5] = 1;
    u[6] = 1;

    std::vector<double> x(u.size());
    x[0] = 1;
    x[1] = 2;
    x[2] = 1;
    x[3] = 1;
    x[4] = 1;
    x[5] = 1;
    x[6] = 1;

    const std::vector<size_t> jacRow;
    const std::vector<size_t> jacCol;
    const std::vector<size_t> hessRow{5, 5, 5, 5, 5};
    const std::vector<size_t> hessCol{0, 1, 3, 4, 6};

    this->testDynamicCustomElements(u, x, jacRow, jacCol, hessRow, hessCol);
}
#endif

TEST_F(CppADCGDynamicTest1, ForwardZero) {
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
