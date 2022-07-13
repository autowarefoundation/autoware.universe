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
#include "CppADCGTest.hpp"

namespace CppAD {
namespace cg {

class CppADCGTempTest : public CppADCGTest {
protected:
    using CGD = CppADCGTest::CGD;
    using ADCGD = CppADCGTest::ADCGD;
public:

    inline CppADCGTempTest(bool verbose = false,
                           bool printValues = false) :
        CppADCGTest(verbose, printValues) {
    }

    void testModel(ADFun<CGD>& f,
                   size_t expectedTmp,
                   size_t expectedArraySize) {
        using CppAD::vector;

        size_t n = f.Domain();
        //size_t m = f.Range();

        CodeHandler<double> handler(10 + n * n);

        vector<CGD> indVars(n);
        handler.makeVariables(indVars);

        vector<CGD> dep = f.Forward(0, indVars);

        LanguageC<double> langC("double");
        LangCDefaultVariableNameGenerator<double> nameGen;

        handler.generateCode(std::cout, langC, dep, nameGen);

        ASSERT_EQ(handler.getTemporaryVariableCount(), expectedTmp);
        ASSERT_EQ(handler.getTemporaryArraySize(), expectedArraySize);
    }
};

} // END cg namespace
} // END CppAD namespace

using namespace CppAD;
using namespace CppAD::cg;

TEST_F(CppADCGTempTest, NoTemporary) {
    size_t n = 3;
    size_t m = 2;

    std::vector<ADCGD> u(n); // independent variable vector
    u[0] = 1;
    u[0] = 2;
    u[0] = 3;
    Independent(u);

    std::vector<ADCGD> Z(m); // dependent variable vector

    // model
    Z[0] = u[0] + u[1];
    ADCGD tmp = u[1] * u[2]; // this temporary variable should disapear
    Z[1] = tmp;

    ADFun<CGD> f(u, Z);
    testModel(f, 0, 0);
}

TEST_F(CppADCGTempTest, Temporary1) {
    size_t n = 3;
    size_t m = 2;

    std::vector<ADCGD> u(n); // independent variable vector
    u[0] = 1;
    u[0] = 2;
    u[0] = 3;
    Independent(u);

    std::vector<ADCGD> Z(m); // dependent variable vector

    // model
    Z[0] = u[0] + u[1];
    ADCGD tmp = u[1] * u[2]; // this temporary variable should NOT disapear
    Z[1] = tmp + tmp;

    ADFun<CGD> f(u, Z);
    testModel(f, 1, 0);
}

TEST_F(CppADCGTempTest, Temporary2) {
    size_t n = 4;
    size_t m = 3;

    std::vector<ADCGD> u(n); // independent variable vector
    u[0] = 1;
    u[0] = 2;
    u[0] = 3;
    Independent(u);

    std::vector<ADCGD> Z(m); // dependent variable vector

    // model
    Z[0] = u[0] + u[1];
    ADCGD tmp = u[1] * u[2];
    ADCGD tmp1 = tmp + 1;
    Z[1] = tmp1 + tmp1;
    Z[2] = Z[1] * u[3];

    ADFun<CGD> f(u, Z);
    testModel(f, 1, 0);
}

TEST_F(CppADCGTempTest, Temporary3) {
    size_t n = 4;
    size_t m = 3;

    std::vector<ADCGD> ind(n); // independent variable vector
    ind[0] = 1;
    ind[0] = 2;
    ind[0] = 3;
    Independent(ind);

    std::vector<ADCGD> dep(m); // dependent variable vector

    // model
    ADCGD tmpu0 = ind[0] + 1;
    ADCGD tmpu1 = ind[1] + 1;
    dep[2] = tmpu0 + tmpu1;
    ADCGD tmp = ind[1] * ind[2];
    ADCGD tmp1 = tmp + dep[2];
    dep[0] = tmp1 / 2.0;
    dep[1] = tmp1 + ind[3];

    ADFun<CGD> f(ind, dep);
    testModel(f, 1, 0);
}
