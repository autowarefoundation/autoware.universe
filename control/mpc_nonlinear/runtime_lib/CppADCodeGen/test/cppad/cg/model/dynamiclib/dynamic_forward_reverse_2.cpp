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
#include "CppADCGTest.hpp"
#include "gccCompilerFlags.hpp"

namespace CppAD {
namespace cg {

#define _MODEL1

class CppADCGDynamicForRevTest : public CppADCGTest {
protected:
    const std::string _modelName;
    const static size_t n;
    const static size_t m;
    std::vector<double> x;
    ADFun<CGD>* _fun;
    std::unique_ptr<DynamicLib<double>> _dynamicLib;
    std::unique_ptr<GenericModel<double>> _model;
public:

    inline CppADCGDynamicForRevTest(bool verbose = false, bool printValues = false) :
        CppADCGTest(verbose, printValues),
        _modelName("model"),
        x(n),
        _fun(nullptr) {
    }

    void SetUp() override {
        // use a special object for source code generation
        using Base = double;
        using CGD = CG<Base>;
        using ADCG = AD<CGD>;

#ifdef _MODEL1
        for (size_t j = 0; j < n; j++)
            x[j] = j + 2;
#else
        x[0] = 0.5;
#endif
        // independent variables
        std::vector<ADCG> u(n);
        for (size_t j = 0; j < n; j++)
            u[j] = x[j];

        CppAD::Independent(u);

        // dependent variable vector 
        std::vector<ADCG> Z(m);

        /**
         * create the CppAD tape as usual
         */
#ifdef _MODEL1
        Z[0] = cos(u[0]);
        Z[1] = u[1] * u[2] + sin(u[0]);
        Z[2] = u[2] * u[2] + sin(u[1]);
        Z[3] = u[0] / u[2] + u[1] * u[2] + 5.0;
#else
        Z[0] = 1.0 / u[0];
#endif
        // create f: U -> Z and vectors used for derivative calculations
        _fun = new ADFun<CGD>(u, Z);

        /**
         * Create the dynamic library
         * (generate and compile source code)
         */
        ModelCSourceGen<double> compHelp(*_fun, _modelName);

        compHelp.setCreateForwardZero(true);
        compHelp.setCreateForwardOne(true);
        compHelp.setCreateReverseOne(true);
        compHelp.setCreateReverseTwo(true);
        compHelp.setCreateSparseJacobian(true);
        compHelp.setCreateSparseHessian(true);

        GccCompiler<double> compiler;
        prepareTestCompilerFlags(compiler);

        ModelLibraryCSourceGen<double> compDynHelp(compHelp);

        DynamicModelLibraryProcessor<double> p(compDynHelp);

        _dynamicLib = p.createDynamicLibrary(compiler);
        _model = _dynamicLib->model(_modelName);

        // dimensions
        ASSERT_EQ(_model->Domain(), _fun->Domain());
        ASSERT_EQ(_model->Range(), _fun->Range());
    }

    void TearDown() override {
        _dynamicLib.reset(nullptr);
        _model.reset(nullptr);
        delete _fun;
        _fun = nullptr;
    }

};

/**
 * static data
 */
#ifdef _MODEL1
const size_t CppADCGDynamicForRevTest::n = 3;
const size_t CppADCGDynamicForRevTest::m = 4;
#else
const size_t CppADCGDynamicForRevTest::n = 1;
const size_t CppADCGDynamicForRevTest::m = 1;
#endif

} // END cg namespace
} // END CppAD namespace

using namespace CppAD;
using namespace CppAD::cg;
using namespace std;

TEST_F(CppADCGDynamicForRevTest, SparseJacobian) {
    using namespace std;
    using std::vector;

    vector<CGD> xOrig(n);
    for (size_t i = 0; i < n; i++)
        xOrig[i] = x[i];

    const std::vector<bool> p = jacobianSparsity < std::vector<bool>, CGD > (*_fun);

    vector<CGD> jacOrig = _fun->SparseJacobian(xOrig, p);
    vector<double> jacCG = CppADCGDynamicForRevTest::_model->SparseJacobian(x);

    ASSERT_TRUE(compareValues(jacCG, jacOrig));
}

TEST_F(CppADCGDynamicForRevTest, SparseHessian) {
    using namespace std;
    using std::vector;

    using Base = double;
    using CGD = CppAD::cg::CG<Base>;

    vector<double> w(m);
    for (size_t i = 0; i < m; i++)
        w[i] = 1;

    vector<CGD> wOrig(m);
    for (size_t i = 0; i < m; i++)
        wOrig[i] = w[i];

    vector<CGD> xOrig(n);
    for (size_t i = 0; i < n; i++)
        xOrig[i] = x[i];

    vector<CGD> hessOrig = _fun->SparseHessian(xOrig, wOrig);
    vector<double> hessCG = CppADCGDynamicForRevTest::_model->SparseHessian(x, w);

    ASSERT_TRUE(compareValues(hessCG, hessOrig));
}
