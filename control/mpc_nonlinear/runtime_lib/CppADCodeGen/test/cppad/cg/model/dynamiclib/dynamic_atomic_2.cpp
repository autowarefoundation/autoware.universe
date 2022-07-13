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

void testModel(const std::vector<AD<double> >& ax, std::vector<AD<double> >& ay) {
    ay[0] = cos(ax[0]);
    ay[1] = ax[1] * ax[2] + sin(ax[0]);
    ay[2] = ax[2] * ax[2] + sin(ax[1]);
    ay[3] = ax[0] / ax[2] + ax[1] * ax[2] + 5.0;
}

/**
 * Test class
 */
class CppADCGDynamicAtomic2Test : public CppADCGTest {
protected:
    const static std::string MODEL_NAME;
    const static size_t m;
    const static size_t n;
    std::vector<double> x;
    checkpoint<double>* _atomicFun;
    CGAtomicFun<double>* _cgAtomicFun;
    ADFun<CGD>* _fun;
    std::unique_ptr<DynamicLib<double>> _dynamicLib;
    std::unique_ptr<GenericModel<double>> _model;
public:

    inline CppADCGDynamicAtomic2Test(bool verbose = false, bool printValues = false) :
        CppADCGTest(verbose, printValues),
        x(n),
        _atomicFun(nullptr),
        _cgAtomicFun(nullptr),
        _fun(nullptr) {
    }

    void SetUp() override {
        // use a special object for source code generation
        using Base = double;
        using CGD = CG<Base>;
        using ADCG = AD<CGD>;
        using ADD = AD<Base>;

        std::vector<ADD> ax(n);
        std::vector<ADD> ay(m);

        for (size_t j = 0; j < n; j++) {
            x[j] = j + 2;
            ax[j] = x[j];
        }

        // independent variables
        std::vector<ADCG> u(n);
        for (size_t j = 0; j < n; j++) {
            u[j] = x[j];
        }

        CppAD::Independent(u);

        // dependent variable vector
        std::vector<ADCG> Z(m);

        _atomicFun = new checkpoint<double>("func", testModel, ax, ay); // the normal atomic function
        _cgAtomicFun = new CGAtomicFun<double>(*_atomicFun, x, true); // a wrapper used to tape with CG<Base>

        (*_cgAtomicFun)(u, Z);

        // create f: U -> Z and vectors used for derivative calculations
        _fun = new ADFun<CGD>(u, Z);

        /**
         * Create the dynamic library
         * (generate and compile source code)
         */
        ModelCSourceGen<double> compHelp(*_fun, MODEL_NAME);

        compHelp.setCreateForwardZero(true);
        compHelp.setCreateForwardOne(true);
        compHelp.setCreateReverseOne(true);
        compHelp.setCreateReverseTwo(true);
        compHelp.setCreateJacobian(true);
        compHelp.setCreateHessian(true);
        compHelp.setCreateSparseJacobian(true);
        compHelp.setCreateSparseHessian(true);

        ModelLibraryCSourceGen<double> compDynHelp(compHelp);

        SaveFilesModelLibraryProcessor<double>::saveLibrarySourcesTo(compDynHelp, MODEL_NAME);

        DynamicModelLibraryProcessor<double> p(compDynHelp);
        GccCompiler<double> compiler;
        prepareTestCompilerFlags(compiler);

        _dynamicLib = p.createDynamicLibrary(compiler);
        _model = _dynamicLib->model(MODEL_NAME);
        _model->addAtomicFunction(*_atomicFun);
    }

    void TearDown() override {
        delete _cgAtomicFun;
        _cgAtomicFun = nullptr;
        delete _atomicFun;
        _atomicFun = nullptr;
        _model.reset(nullptr);
        _dynamicLib.reset(nullptr);
        delete _fun;
        _fun = nullptr;
    }

};

/**
 * static data
 */
const std::string CppADCGDynamicAtomic2Test::MODEL_NAME = "dynamicAtomic2";
const size_t CppADCGDynamicAtomic2Test::n = 3;
const size_t CppADCGDynamicAtomic2Test::m = 4;

} // END cg namespace
} // END CppAD namespace

using namespace CppAD;
using namespace CppAD::cg;
using namespace std;

TEST_F(CppADCGDynamicAtomic2Test, DynamicForRev) {
    using CppAD::vector;

    using Base = double;
    using CGD = CppAD::cg::CG<Base>;

    vector< AD<double> > ax(n);
    for (size_t j = 0; j < n; j++)
        ax[j] = x[j];

    // declare independent variables and start tape recording
    CppAD::Independent(ax);

    /**
     * Test zero order
     */
    vector<CGD> xOrig(n);
    for (size_t j = 0; j < n; j++)
        xOrig[j] = x[j];

    vector<CGD> yOrig = _fun->Forward(0, xOrig);
    std::vector<double> yInner = _model->ForwardZero(x);

    ASSERT_TRUE(compareValues(yInner, yOrig));

    /**
     * Test first order forward mode
     */
    size_t k = 1;
    size_t k1 = k + 1;

    vector<double> x_p(n);
    for (size_t j = 0; j < n; j++)
        x_p[j] = 0;

    vector<CGD> x_pOrig(n);
    vector<double> tx(k1 * n);
    for (size_t j = 0; j < n; j++)
        tx[j * k1] = x[j]; // zero order
    for (size_t j = 0; j < n; j++)
        tx[j * k1 + 1] = 0; // first order

    for (size_t j = 0; j < n; j++) {
        x_p[j] = 1;
        x_pOrig[j] = 1;
        tx[j * k1 + 1] = 1;

        vector<CGD> y_pOrig = _fun->Forward(1, x_pOrig);
        vector<double> y_pInner = _model->ForwardOne(tx);

        x_p[j] = 0;
        x_pOrig[j] = 0;
        tx[j * k1 + 1] = 0;

        ASSERT_TRUE(compareValues(y_pInner, y_pOrig));
    }

    /**
     * Test first order reverse mode
     */
    k = 0;
    k1 = k + 1;

    CppAD::vector<double> w(m);
    std::vector<double> stdw(m);
    for (size_t i = 0; i < m; i++)
        w[i] = 0;
    vector<CGD> wOrig(m);
    tx.resize(k1 * n);
    for (size_t j = 0; j < n; j++)
        tx[j * k1] = x[j]; // zero order
    vector<double> ty(k1 * m);
    for (size_t i = 0; i < m; i++)
        ty[i * k1] = yInner[i]; // zero order

    for (size_t i = 0; i < m; i++) {
        w[i] = 1;
        wOrig[i] = 1;

        vector<CGD> dwOrig = _fun->Reverse(1, wOrig);
        vector<double> dwInner = _model->ReverseOne(tx, ty, w);

        w[i] = 0;
        wOrig[i] = 0;

        ASSERT_TRUE(compareValues(dwInner, dwOrig));
    }

    /**
     * Test second order reverse mode
     */
    k = 1;
    k1 = k + 1;
    tx.resize(k1 * n);
    ty.resize(k1 * m);
    vector<double> py(k1 * m);
    vector<CGD> pyOrig(k1 * m);
    //wOrig.resize(k1 * m);
    for (size_t j = 0; j < n; j++) {
        tx[j * k1] = x[j]; // zero order
        tx[j * k1 + 1] = 0; // first order
    }
    for (size_t i = 0; i < m; i++) {
        ty[i * k1] = yInner[i]; // zero order
        ty[i * k1 + 1] = 0; // first order
        py[i * k1] = 0.0;
        py[i * k1 + 1] = 1.0; // first order
        pyOrig[i * k1] = 0.0;
        pyOrig[i * k1 + 1] = 1.0; // first order
    }

    for (size_t j = 0; j < n; j++) {
        x_p[j] = 1;
        x_pOrig[j] = 1;
        tx[j * k1 + 1] = 1;

        _fun->Forward(1, x_pOrig);
        vector<CGD> pxOrig = _fun->Reverse(2, pyOrig);
        vector<double> pxInner = _model->ReverseTwo(tx, ty, py);

        x_p[j] = 0;
        x_pOrig[j] = 0;
        tx[j * k1 + 1] = 0;

        // only compare second order information
        // (location of the elements is different then if py.size() == m)
        ASSERT_EQ(pxOrig.size(), n * k1);
        ASSERT_EQ(pxOrig.size(), pxInner.size());
        for (size_t j2 = 0; j2 < n; j2++) {
            ASSERT_TRUE(nearEqual(pxInner[j2 * k1], pxOrig[j2 * k1].getValue()));
        }
    }

    /**
     * Jacobian
     */
    vector<CGD> jacOrig = _fun->Jacobian(xOrig);
    std::vector<double> jacOuter = _model->Jacobian(x);
    ASSERT_TRUE(compareValues(jacOuter, jacOrig));

    /**
     * Jacobian sparsity
     */
    const std::vector<bool> jacSparsityOrig = jacobianForwardSparsity < std::vector<bool>, CGD > (*_fun);
    const std::vector<bool> jacSparsityOuter = _model->JacobianSparsityBool();

    compareBoolValues(jacSparsityOrig, jacSparsityOuter);

    /**
     * Sparse jacobian
     */
    jacOrig = _fun->SparseJacobian(xOrig);
    jacOuter = _model->SparseJacobian(x);
    ASSERT_TRUE(compareValues(jacOuter, jacOrig));

    // sparse reverse
    std::vector<size_t> row, col;
    generateSparsityIndexes(jacSparsityOuter, m, n, row, col);

    jacOuter.resize(row.size());
    _model->SparseJacobian(x, jacOuter, row, col);

    sparse_jacobian_work workOrig;
    jacOrig.resize(row.size());
    _fun->SparseJacobianReverse(xOrig, jacSparsityOrig, row, col, jacOrig, workOrig);

    ASSERT_TRUE(compareValues(jacOuter, jacOrig));

    /**
     * Hessian
     */
    for (size_t i = 0; i < m; i++) {
        stdw[i] = 1;
        wOrig[i] = 1;
    }
    vector<CGD> hessOrig = _fun->Hessian(xOrig, wOrig);
    std::vector<double> hessOuter = _model->Hessian(x, stdw);
    ASSERT_TRUE(compareValues(hessOuter, hessOrig));

    /**
     * Sparse Hessian
     */
    hessOrig = _fun->SparseHessian(xOrig, wOrig);
    hessOuter = _model->SparseHessian(x, stdw);
    ASSERT_TRUE(compareValues(hessOuter, hessOrig));
}
