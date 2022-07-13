#ifndef CPPAD_CG_TEST_CPPADCGDYNAMICATOMICTEST_INCLUDED
#define CPPAD_CG_TEST_CPPADCGDYNAMICATOMICTEST_INCLUDED
/* --------------------------------------------------------------------------
 *  CppADCodeGen: C++ Algorithmic Differentiation with Source Code Generation:
 *    Copyright (C) 2013 Ciengis
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

#include "CppADCGModelTest.hpp"
#include "gccCompilerFlags.hpp"

namespace CppAD {
namespace cg {

class CppADCGDynamicAtomicTest : public CppADCGModelTest {
public:
    using Super = CppADCGTest;
    using Base = Super::Base;
    using CGD = Super::CGD;
    using ADCGD = Super::ADCGD;
protected:
    const std::string _modelName;
    std::unique_ptr<ADFun<CGD>> _funInner; // inner model tape
    std::unique_ptr<ADFun<CGD>> _funOuter; // outer model tape with no atomics
    std::unique_ptr<ADFun<CGD>> _funOuterAtom; // outer model tape with atomics
    std::unique_ptr<ADFun<Base>> _funBaseOuterAtom; // outer model tape with atomics
    std::unique_ptr<DynamicLib<Base>> _dynamicLib; // library for the inner model (in may also have the outer model)
    std::unique_ptr<DynamicLib<Base>> _dynamicLibOuter; // library for the outer model
    std::unique_ptr<GenericModel<Base>> _modelLib;
    std::unique_ptr<CGAtomicFun<Base>> _atomFun;
public:

    explicit CppADCGDynamicAtomicTest(std::string modelName,
                                      bool verbose = false,
                                      bool printValues = false) :
            CppADCGModelTest(verbose, printValues),
            _modelName(std::move(modelName)) {
        //this->verbose_ = true;
    }

    virtual std::vector<ADCGD> model(const std::vector<ADCGD>& x) = 0;

    virtual std::vector<ADCGD> modelOuter(const std::vector<ADCGD>& y) {
        std::vector<ADCGD> z(y.size() - 1);

        for (size_t i = 0; i < y.size() - 1; i++) {
            z[i] = 2 * y[i];
        }
        z[z.size() - 1] += y[y.size() - 1];

        return z;
    }

    void TearDown() override {
        // need to clear memory here because CppAD checks if there is any memory still in use
        _dynamicLib.reset();
        _dynamicLibOuter.reset();
        _modelLib.reset();
        _funInner.reset();
        _funOuter.reset();
        _funOuterAtom.reset();
        _funBaseOuterAtom.reset();
        _atomFun.reset();
    }

    ~CppADCGDynamicAtomicTest() override = default;

    inline static CppAD::vector<Base> makeValues(size_t n) {
        CppAD::vector<Base> x(n);

        for (size_t j = 0; j < n; j++)
            x[j] = j + 2;

        return x;
    }

    /**
     * Tests one compiled model used as an atomic function by an ADFun.
     * The outer model method (modelOuter) is NOT used here!
     * It uses z = g(x) = f(x).
     */
    void testADFunAtomicLibSimple(const CppAD::vector<Base>& x,
                                  const CppAD::vector<Base>& xNorm={},
                                  const CppAD::vector<Base>& eqNorm = {},
                                  Base epsilonR = 1e-14, Base epsilonA = 1e-14) {
        ASSERT_TRUE(xNorm.size() == 0 || x.size() == xNorm.size());

        using namespace CppAD;
        using namespace std;
        using CppAD::vector;

        tapeInnerModel(x, xNorm, eqNorm);

        unique_ptr<GenericModel<Base> > modelLib = compileInnerModel("");

        const size_t n = _funInner->Domain();
        const size_t m = _funInner->Range();

        vector<AD<double> > ax(n);
        for (size_t j = 0; j < n; j++)
            ax[j] = x[j];

        // declare independent variables and start tape recording
        CppAD::Independent(ax);

        vector<AD<double> > ay(m);

        // call user function and store CGAtomicLibModel(x) in au[0]
        CGAtomicGenericModel<double>& atomicfun = modelLib->asAtomic();

        atomicfun(ax, ay);

        // create fWrapAtom: x -> y and stop tape recording
        ADFun<double> fWrapAtom(ax, ay);

        /**
         * Test zero order
         */
        std::vector<Base> xOrig(x.data(), x.data() + x.size());

        testForwardZeroResults(*modelLib, *_funInner, &fWrapAtom, xOrig, epsilonR, epsilonA);

        /**
         * Test first order forward mode
         */
        testForwardOneResults(*modelLib, *_funInner, &fWrapAtom, xOrig, epsilonR, epsilonA);

        /**
         * Test first order reverse mode
         */
        testReverseOneResults(*modelLib, *_funInner, &fWrapAtom, xOrig, epsilonR, epsilonA);

        /**
         * Test second order reverse mode
         */
        testReverseTwoResults(*modelLib, *_funInner, &fWrapAtom, xOrig, epsilonR, epsilonA);

        /**
         * Dense Jacobian
         */
        testDenseJacResults(*modelLib, *_funInner, xOrig, epsilonR, epsilonA);

        /**
         * Jacobian sparsity
         */
        testJacobianSparsity(*_funInner, fWrapAtom);

        /**
         * Sparse Jacobian
         */
        size_t n_tests = _dynamicLib->getThreadNumber() > 1 ? 2 : 1;
        testSparseJacobianResults(n_tests, *modelLib, *_funInner, &fWrapAtom, xOrig, false, epsilonR, epsilonA);

        /**
         * Dense Hessian
         */
        testDenseHessianResults(*modelLib, *_funInner, xOrig, epsilonR, epsilonA);

        /**
         * Sparse Hessian
         */
        testSparseHessianResults(n_tests, *modelLib, *_funInner, &fWrapAtom, xOrig, false, epsilonR, epsilonA);
    }

    /**
     * Tests one compiled model used as an atomic function by an ADFun.
     * The outer model method (modelOuter) is used!
     * It uses z = g(f(x)).
     */
    void testADFunAtomicLib(const CppAD::vector<Base>& x,
                            const CppAD::vector<Base>& xNorm,
                            const CppAD::vector<Base>& eqNorm,
                            Base epsilonR = 1e-14,
                            Base epsilonA = 1e-14) {
        prepareAtomicLibAtomicLib(x, xNorm, eqNorm);
        ASSERT_TRUE(_modelLib != nullptr);
        ASSERT_TRUE(_funOuter != nullptr);

        auto atomic = [&](const std::vector<ADCGD>& ax,
                          std::vector<ADCGD>& ay){
            assert(_modelLib != nullptr);
            assert(_modelLib->getName() ==  _modelName);
            _atomFun.reset(new CGAtomicFun<double> (_modelLib->asAtomic(), x));

            (*_atomFun)(ax, ay); // atomic function
        };

        tapeOuterModelWithAtomic(atomic, x, xNorm, eqNorm);
        ASSERT_TRUE(_funOuterAtom != nullptr);

        testAtomicLibModelInCppAD(*_funOuter, *_funOuterAtom, x, epsilonR, epsilonA);
    }

    void testADFunAtomicLib(const CppAD::vector<Base>& x,
                            Base epsilonR = 1e-14,
                            Base epsilonA = 1e-14) {
        CppAD::vector<Base> xNorm(x.size());
        for (double & i : xNorm)
            i = 1.0;
        CppAD::vector<Base> eqNorm;

        testADFunAtomicLib(x, xNorm, eqNorm, epsilonR, epsilonA);
    }

    /**
     * Test 2 models in 2 dynamic libraries
     */
    void testAtomicLibAtomicLib(const CppAD::vector<Base>& x,
                                Base epsilonR = 1e-14,
                                Base epsilonA = 1e-14) {
        CppAD::vector<Base> xNorm(x.size());
        for (double & i : xNorm)
            i = 1.0;
        CppAD::vector<Base> eqNorm;

        testAtomicLibAtomicLib(x, xNorm, eqNorm, epsilonR, epsilonA);
    }

    /**
     * Test 2 models in 2 dynamic libraries
     */
    void testAtomicLibAtomicLib(const CppAD::vector<Base>& x,
                                const CppAD::vector<Base>& xNorm,
                                const CppAD::vector<Base>& eqNorm,
                                Base epsilonR = 1e-14, Base epsilonA = 1e-14) {
        using namespace std;

        prepareAtomicLibAtomicLib(x, xNorm, eqNorm);
        ASSERT_TRUE(_modelLib != nullptr);

        unique_ptr<GenericModel<Base> > modelLibOuter = _dynamicLibOuter->model(_modelName + "_outer");
        ASSERT_TRUE(modelLibOuter != nullptr);

        test2LevelAtomicLibModel(_modelLib.get(), modelLibOuter.get(),
                                 x, xNorm, eqNorm, epsilonR, epsilonA);
    }

    /**
     * Test 2 models in the same dynamic library
     */
    void testAtomicLibModelBridge(const CppAD::vector<Base>& x,
                                  const CppAD::vector<Base>& xNorm,
                                  const CppAD::vector<Base>& eqNorm,
                                  Base epsilonR = 1e-14, Base epsilonA = 1e-14) {
        using namespace std;

        prepareAtomicLibModelBridge(x, xNorm, eqNorm);

        unique_ptr<GenericModel<Base> > modelLib = _dynamicLib->model(_modelName);
        unique_ptr<GenericModel<Base> > modelLibOuter = _dynamicLib->model(_modelName + "_outer");

        test2LevelAtomicLibModel(modelLib.get(), modelLibOuter.get(),
                                 x, xNorm, eqNorm, epsilonR, epsilonA);
    }

    /**
     * Test 2 models in the same dynamic library
     */
    void testAtomicLibModelBridgeCustom(const CppAD::vector<Base>& x,
                                        const CppAD::vector<Base>& xNorm,
                                        const CppAD::vector<Base>& eqNorm,
                                        const std::vector<std::set<size_t> >& jacInner,
                                        const std::vector<std::set<size_t> >& hessInner,
                                        const std::vector<std::set<size_t> >& jacOuter,
                                        const std::vector<std::set<size_t> >& hessOuter,
                                        bool createOuterReverse2,
                                        Base epsilonR = 1e-14, Base epsilonA = 1e-14) {
        using namespace std;

        prepareAtomicLibModelBridge(x, xNorm, eqNorm,
                                    jacInner, hessInner,
                                    jacOuter, hessOuter,
                                    createOuterReverse2);

        unique_ptr<GenericModel<Base> > modelLib = _dynamicLib->model(_modelName);
        unique_ptr<GenericModel<Base> > modelLibOuter = _dynamicLib->model(_modelName + "_outer");

        test2LevelAtomicLibModelCustomEls(modelLib.get(), modelLibOuter.get(),
                                          x, xNorm, eqNorm,
                                          jacOuter, hessOuter,
                                          epsilonR, epsilonA);
    }

    /**
     * Test the Jacobian and Hessian sparsity patterns computed directly with
     * the methods of the CGAbstractAtomicFun.
     */
    void testAtomicSparsities(const CppAD::vector<Base>& x) {
        CGAtomicFunBridge<double> atomicfun("innerModel", *_funInner, true);

        //const size_t n = _funInner->Domain();
        const size_t m = _funInner->Range();

        CppAD::vector<CGD> xx(x.size());
        for (size_t i = 0; i < x.size(); ++i) {
            xx[i] = x[i];
        }

        CppAD::vector<std::set<size_t>> jacOrig;
        CppAD::vector<std::set<size_t>> jacAtom;

        jacOrig = jacobianForwardSparsitySet<CppAD::vector<std::set<size_t>>, CGD> (*_funInner);
        jacAtom = atomicfun.jacobianForwardSparsitySet(m, xx);
        compareVectorSetValues(jacOrig, jacAtom);

        jacOrig = jacobianReverseSparsitySet<CppAD::vector<std::set<size_t>>, CGD> (*_funInner);
        jacAtom = atomicfun.jacobianReverseSparsitySet(m, xx);
        compareVectorSetValues(jacOrig, jacAtom);

        CppAD::vector<std::set<size_t>> hessOrig;
        CppAD::vector<std::set<size_t>> hessAtom;

        hessOrig = hessianSparsitySet<CppAD::vector<std::set<size_t>>, CGD> (*_funInner);
        hessAtom = atomicfun.hessianSparsitySet(m, xx);
        compareVectorSetValues(hessOrig, hessAtom);
    }

private:

    void test2LevelAtomicLibModel(GenericModel<Base>* modelLib,
                                  GenericModel<Base>* modelLibOuter,
                                  const CppAD::vector<Base>& x,
                                  const CppAD::vector<Base>& xNorm,
                                  const CppAD::vector<Base>& eqNorm,
                                  Base epsilonR = 1e-14, Base epsilonA = 1e-14) {
        ASSERT_EQ(x.size(), xNorm.size());

        using namespace CppAD;
        using namespace std;
        using CppAD::vector;

        std::cout << std::endl << "2LevelAtomicLib" << std::endl;

        const size_t n = _funOuter->Domain();
        const size_t m = _funOuter->Range();

        modelLibOuter->addAtomicFunction(modelLib->asAtomic());
        //modelLibOuter->addExternalModel(*modelLib);


        /**
         * Test zero order
         */
        vector<CGD> xOrig(n);
        for (size_t j = 0; j < n; j++)
            xOrig[j] = x[j];

        vector<CGD> yOrig = _funOuter->Forward(0, xOrig);
        vector<double> yOuter = modelLibOuter->ForwardZero(x);

        ASSERT_TRUE(compareValues<double>(yOuter, yOrig, epsilonR, epsilonA));

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
            x_pOrig[j] = 1;
            tx[j * k1 + 1] = 1;

            vector<CGD> y_pOrig = _funOuter->Forward(1, x_pOrig);
            vector<double> y_pOuter = modelLibOuter->ForwardOne(tx);

            x_pOrig[j] = 0;
            tx[j * k1 + 1] = 0;

            ASSERT_TRUE(compareValues<double>(y_pOuter, y_pOrig, epsilonR, epsilonA));
        }

        /**
         * Test first order reverse mode
         */
        k = 0;
        k1 = k + 1;

        vector<double> w(m);
        for (size_t i = 0; i < m; i++)
            w[i] = 0;
        vector<CGD> wOrig(m);
        tx.resize(k1 * n);
        for (size_t j = 0; j < n; j++)
            tx[j * k1] = x[j]; // zero order
        vector<double> ty(k1 * m);
        for (size_t i = 0; i < m; i++)
            ty[i * k1] = yOuter[i]; // zero order

        for (size_t i = 0; i < m; i++) {
            w[i] = 1;
            wOrig[i] = 1;

            vector<CGD> dwOrig = _funOuter->Reverse(1, wOrig);
            vector<double> dwOuter = modelLibOuter->ReverseOne(tx, ty, w);

            w[i] = 0;
            wOrig[i] = 0;

            ASSERT_TRUE(compareValues<double>(dwOuter, dwOrig, epsilonR, epsilonA));
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
            ty[i * k1] = yOuter[i]; // zero order
            py[i * k1] = 0.0;
            py[i * k1 + 1] = 1.0; // first order
            pyOrig[i * k1] = 0.0;
            pyOrig[i * k1 + 1] = 1.0; // first order
        }

        for (size_t j = 0; j < n; j++) {
            x_pOrig[j] = 1;
            tx[j * k1 + 1] = 1;

            _funOuter->Forward(1, x_pOrig);
            vector<CGD> dwOrig = _funOuter->Reverse(2, pyOrig);
            vector<double> dwOuter = modelLibOuter->ReverseTwo(tx, ty, py);

            x_pOrig[j] = 0;
            tx[j * k1 + 1] = 0;

            // only compare second order information
            // (location of the elements is different then if py.size() == m)
            ASSERT_EQ(dwOrig.size(), n * k1);
            ASSERT_EQ(dwOrig.size(), dwOuter.size());
            for (size_t j2 = 0; j2 < n; j2++) {
                ASSERT_TRUE(nearEqual(dwOuter[j2 * k1], dwOrig[j2 * k1].getValue()));
            }
        }

        /**
         * Jacobian
         */
        vector<CGD> jacOrig = _funOuter->Jacobian(xOrig);
        vector<double> jacOuter = modelLibOuter->Jacobian(x);
        ASSERT_TRUE(compareValues<double>(jacOuter, jacOrig, epsilonR, epsilonA));

        /**
         * Jacobian sparsity
         */
        const std::vector<bool> jacSparsityOrig = jacobianSparsity<std::vector<bool>, CGD> (*_funOuter);
        const std::vector<bool> jacSparsityOuter = modelLibOuter->JacobianSparsityBool();

        compareBoolValues(jacSparsityOrig, jacSparsityOuter);

        /**
         * Sparse jacobian
         */
        jacOrig = _funOuter->SparseJacobian(xOrig);
        jacOuter = modelLibOuter->SparseJacobian(x);
        ASSERT_TRUE(compareValues<double>(jacOuter, jacOrig, epsilonR, epsilonA));

        /**
         * Hessian
         */
        for (size_t i = 0; i < m; i++) {
            w[i] = 1;
            wOrig[i] = 1;
        }
        vector<CGD> hessOrig = _funOuter->Hessian(xOrig, wOrig);
        vector<double> hessOuter = modelLibOuter->Hessian(x, w);
        ASSERT_TRUE(compareValues<double>(hessOuter, hessOrig, epsilonR, epsilonA));

        /**
         * Hessian sparsity
         */
        const std::vector<bool> hessSparsityOrig = hessianSparsity<std::vector<bool>, CGD> (*_funOuter);
        const std::vector<bool> hessSparsityOuter = modelLibOuter->HessianSparsityBool();

        compareBoolValues(hessSparsityOrig, hessSparsityOuter);

        /**
         * Sparse Hessian
         */
        hessOrig = _funOuter->SparseHessian(xOrig, wOrig);
        hessOuter = modelLibOuter->SparseHessian(x, w);
        ASSERT_TRUE(compareValues<double>(hessOuter, hessOrig, epsilonR, epsilonA));
    }

    static void testAtomicLibModelInCppAD(ADFun<CGD>& funOuter,
                                          ADFun<CGD>& funOuterAtom, // with an atomic function
                                          const CppAD::vector<Base>& xx,
                                          Base epsilonR = 1e-14, Base epsilonA = 1e-14) {
        using namespace CppAD;
        using namespace std;
        using CppAD::vector;

        std::cout << std::endl << "AtomicLibModelInCppAD" << std::endl;

        const size_t n = funOuter.Domain();
        const size_t m = funOuter.Range();

        /**
         * Test zero order
         */
        vector<CGD> x(n);
        for (size_t j = 0; j < n; j++)
            x[j] = xx[j];

        vector<CGD> y = funOuter.Forward(0, x);
        vector<CGD> yAtom = funOuterAtom.Forward(0, x); // with an atomic function

        ASSERT_TRUE(compareValues<double>(y, yAtom, epsilonR, epsilonA));

        /**
         * Test first order forward mode
         */

        vector<CGD> x_p(n);

        for (size_t j = 0; j < n; j++) {
            x_p[j] = 1;

            vector<CGD> y_p = funOuter.Forward(1, x_p);
            vector<CGD> y_pAtom = funOuterAtom.Forward(1, x_p); // with an atomic function

            x_p[j] = 0;

            ASSERT_TRUE(compareValues<double>(y_p, y_pAtom, epsilonR, epsilonA));
        }

        /**
         * Test first order reverse mode
         */
        vector<CGD> w(m);
        for (size_t i = 0; i < m; i++) {
            w[i] = 1;

            vector<CGD> dw = funOuter.Reverse(1, w);
            vector<CGD> dwAtom = funOuterAtom.Reverse(1, w); // with an atomic function

            w[i] = 0;

            ASSERT_TRUE(compareValues<double>(dw, dwAtom, epsilonR, epsilonA));
        }

        /**
         * Test second order reverse mode
         */
        size_t k = 1;
        size_t k1 = k + 1;
        vector<CGD> py(m); // not (k1 * m)
        for (size_t i = 0; i < m; i++) {
            //py[i * k1] = 0.0;
            //py[i * k1 + 1] = 1.0; // first order
            py[i] = 1.0; // first order
        }

        for (size_t j = 0; j < n; j++) {
            x_p[j] = 1;

            funOuter.Forward(1, x_p);
            funOuterAtom.Forward(1, x_p);  // with an atomic function
            vector<CGD> dw = funOuter.Reverse(2, py);
            vector<CGD> dwAtom = funOuterAtom.Reverse(2, py); // with an atomic function

            x_p[j] = 0;

            // only compare second order information
            ASSERT_EQ(dw.size(), n * k1);
            ASSERT_EQ(dw.size(), dwAtom.size());
            for (size_t j2 = 0; j2 < n; j2++) {
                ASSERT_TRUE(nearEqual(dw[j2 * k1 + 1].getValue(), dwAtom[j2 * k1 + 1].getValue()));
            }
        }

        /**
         * Jacobian
         */
        vector<CGD> jac = funOuter.Jacobian(x);
        vector<CGD> jacAtom = funOuterAtom.Jacobian(x); // with an atomic function
        ASSERT_TRUE(compareValues<double>(jac, jacAtom, epsilonR, epsilonA));

        /**
         * Jacobian sparsity
         */
        const std::vector<bool> jacSparsity = jacobianSparsity<std::vector<bool>, CGD> (funOuter);
        const std::vector<bool> jacSparsityAtom = jacobianSparsity<std::vector<bool>, CGD> (funOuterAtom);

        compareBoolValues(jacSparsity, jacSparsityAtom);

        /**
         * Sparse jacobian
         */
        jac = funOuter.SparseJacobian(x);
        jacAtom = funOuterAtom.SparseJacobian(x); // with an atomic function
        ASSERT_TRUE(compareValues<double>(jac, jacAtom, epsilonR, epsilonA));

        /**
         * Hessian
         */
        for (size_t i = 0; i < m; i++) {
            w[i] = 1;
        }
        vector<CGD> hess = funOuter.Hessian(x, w);
        vector<CGD> hessAtom = funOuterAtom.Hessian(x, w); // with an atomic function
        ASSERT_TRUE(compareValues<double>(hess, hessAtom, epsilonR, epsilonA));

        /**
         * Hessian sparsity
         */
        const std::vector<bool> hessSparsity = hessianSparsity<std::vector<bool>, CGD> (funOuter);
        const std::vector<bool> hessSparsityAtom = hessianSparsity<std::vector<bool>, CGD> (funOuterAtom);

        compareBoolValues(hessSparsity, hessSparsityAtom);

        /**
         * Sparse Hessian
         */
        hess = funOuter.SparseHessian(x, w);
        hessAtom = funOuterAtom.SparseHessian(x, w); // with an atomic function
        ASSERT_TRUE(compareValues<double>(hess, hessAtom, epsilonR, epsilonA));

        //CodeHandler<Base> handler;
        //handler.makeVariables(x);
        //handler.makeVariables(w);
        //hessAtom = funOuterAtom.SparseHessian(x, w); // with an atomic function
        //
        //LanguageC<Base> lang("double");
        //LangCDefaultVariableNameGenerator<Base> nameGen("hess");
        //LangCDefaultHessianVarNameGenerator<Base> nameGenHess(&nameGen, n);
        //handler.generateCode(std::cout, lang, hessAtom, nameGenHess);
        //std::cout << std::endl;
    }

    void test2LevelAtomicLibModelCustomEls(GenericModel<Base>* modelLib,
                                           GenericModel<Base>* modelLibOuter,
                                           const CppAD::vector<Base>& x,
                                           const CppAD::vector<Base>& xNorm,
                                           const CppAD::vector<Base>& eqNorm,
                                           const std::vector<std::set<size_t> >& jacOuterSpar,
                                           const std::vector<std::set<size_t> >& hessOuterSpar,
                                           Base epsilonR = 1e-14, Base epsilonA = 1e-14) {
        ASSERT_EQ(x.size(), xNorm.size());

        using namespace CppAD;
        using namespace std;
        using CppAD::vector;

        const size_t n = _funOuter->Domain();
        const size_t m = _funOuter->Range();

        modelLibOuter->addAtomicFunction(modelLib->asAtomic());

        /**
         * Test zero order
         */
        vector<CGD> xOrig(n);
        for (size_t j = 0; j < n; j++)
            xOrig[j] = x[j];

        vector<CGD> yOrig = _funOuter->Forward(0, xOrig);
        vector<double> yOuter = modelLibOuter->ForwardZero(x);

        ASSERT_TRUE(compareValues<double>(yOuter, yOrig, epsilonR, epsilonA));

        /**
         * Jacobian sparsity
         */
        const std::vector<bool> jacSparsityOrig = jacobianSparsity<std::vector<bool>, CGD> (*_funOuter);

        /**
         * Sparse jacobian
         */
        CppAD::sparse_jacobian_work work; // temporary structure for CPPAD
        std::vector<size_t> jacOuterRows, jacOuterCols;
        generateSparsityIndexes(jacOuterSpar, jacOuterRows, jacOuterCols);
        vector<CGD> jacOrig(jacOuterCols.size());
        _funOuter->SparseJacobianReverse(xOrig, jacSparsityOrig,
                                         jacOuterRows, jacOuterCols, jacOrig, work);

        std::vector<double> jacOuter(jacOuterRows.size());
        size_t const* rows, *cols;
        modelLibOuter->SparseJacobian(x, jacOuter, &rows, &cols);

        ASSERT_TRUE(compareValues<double>(jacOuter, jacOrig, epsilonR, epsilonA));

        /**
         * Hessian sparsity
         */
        const std::vector<bool> hessSparsityOrig = hessianSparsity<std::vector<bool>, CGD> (*_funOuter);
        /**
        printSparsityPattern(hessSparsityOrig, "original", n, n);

        std::vector<size_t> hessOuterRows2, hessOuterCols2;
        modelLibOuter->HessianSparsity(hessOuterRows2, hessOuterCols2);
        printSparsityPattern(hessOuterRows2, hessOuterCols2, "outer", n);
        */

        /**
         * Sparse Hessian
         */
        vector<CGD> wOrig(m);
        vector<double> w(m);
        for (size_t i = 0; i < m; i++) {
            wOrig[i] = 1;
            w[i] = 1;
        }
        CppAD::sparse_hessian_work hessWork;
        std::vector<size_t> hessOuterRows, hessOuterCols;
        generateSparsityIndexes(hessOuterSpar, hessOuterRows, hessOuterCols);
        vector<CGD> hessOrig(hessOuterRows.size());
        _funOuter->SparseHessian(xOrig, wOrig, hessSparsityOrig,
                                 hessOuterRows, hessOuterCols, hessOrig, hessWork);

        vector<double> hessOuter(hessOuterRows.size());
        modelLibOuter->SparseHessian(x, w,
                                     hessOuter, &rows, &cols);

        ASSERT_TRUE(compareValues<double>(hessOuter, hessOrig, epsilonR, epsilonA));
    }

    virtual void prepareAtomicLibAtomicLib(const CppAD::vector<Base>& x,
                                           const CppAD::vector<Base>& xNorm,
                                           const CppAD::vector<Base>& eqNorm) {
        using CppAD::vector;

        tapeInnerModel(x, xNorm, eqNorm);

        const size_t m = _funInner->Range();
        const size_t n = _funInner->Domain();

        /**
         * Create the dynamic library model
         */
        auto cSourceInner = prepareInnerModelCompilation();

        /**
         * Create the dynamic library
         * (generate and compile source code)
         */
        GccCompiler<double> compiler1;
        prepareTestCompilerFlags(compiler1);
        compiler1.setSourcesFolder("sources_atomiclibatomiclib_" + _modelName);
        compiler1.setSaveToDiskFirst(true);

        ModelLibraryCSourceGen<double> compDynHelp(*cSourceInner);
        compDynHelp.setVerbose(this->verbose_);

        DynamicModelLibraryProcessor<double> p(compDynHelp, "innerModel");
        _dynamicLib = p.createDynamicLibrary(compiler1);
        _modelLib = _dynamicLib->model(_modelName);

        /**
         * Second compiled model
         */
        // independent variables
        std::vector<ADCGD> ax2(n);
        for (size_t j = 0; j < n; j++)
            ax2[j] = x[j];

        CppAD::Independent(ax2);

        CGAtomicGenericModel<Base>& innerAtomicFun = _modelLib->asAtomic();
        CGAtomicFun<Base> cgInnerAtomicFun(innerAtomicFun, std::vector<double>(_modelLib->Domain()), true); // required for taping

        std::vector<ADCGD> ZZ(m);
        cgInnerAtomicFun(ax2, ZZ);
        std::vector<ADCGD> Z2 = modelOuter(ZZ);

        // create f: U2 -> Z2
        ADFun<CGD> fun2;
        fun2.Dependent(Z2);

        ModelCSourceGen<double> cSourceOuter(fun2, _modelName + "_outer");
        cSourceOuter.setCreateForwardZero(true);
        cSourceOuter.setCreateForwardOne(true);
        cSourceOuter.setCreateReverseOne(true);
        cSourceOuter.setCreateReverseTwo(true);
        cSourceOuter.setCreateJacobian(true);
        cSourceOuter.setCreateHessian(true);
        cSourceOuter.setCreateSparseJacobian(true);
        cSourceOuter.setCreateSparseHessian(true);
        //cSourceOuter.setMaxAssignmentsPerFunc(20);

        /**
         * Create the dynamic library
         * (generate and compile source code)
         */
        ModelLibraryCSourceGen<double> compDynHelp2(cSourceOuter);
        compDynHelp2.setVerbose(this->verbose_);

        DynamicModelLibraryProcessor<double> p2(compDynHelp2, "outerModel");
        GccCompiler<double> compiler2;
        prepareTestCompilerFlags(compiler2);
        compiler2.setSourcesFolder("sources_atomiclibatomiclib_" + _modelName);
        compiler2.setSaveToDiskFirst(true);
        _dynamicLibOuter = p2.createDynamicLibrary(compiler2);

        /**
         * tape the model without atomics
         */
        tapeOuterModel(x, xNorm, eqNorm);
    }

    virtual void prepareAtomicLibModelBridge(const CppAD::vector<Base>& x,
                                             const CppAD::vector<Base>& xNorm,
                                             const CppAD::vector<Base>& eqNorm) {
        std::vector<std::set<size_t> > jacInner, hessInner;
        std::vector<std::set<size_t> > jacOuter, hessOuter;
        prepareAtomicLibModelBridge(x, xNorm, eqNorm,
                                    jacInner, hessInner,
                                    jacOuter, hessOuter,
                                    true);
    }

    virtual void prepareAtomicLibModelBridge(const CppAD::vector<Base>& x,
                                             const CppAD::vector<Base>& xNorm,
                                             const CppAD::vector<Base>& eqNorm,
                                             const std::vector<std::set<size_t> >& jacInner,
                                             const std::vector<std::set<size_t> >& hessInner,
                                             const std::vector<std::set<size_t> >& jacOuter,
                                             const std::vector<std::set<size_t> >& hessOuter,
                                             bool createOuterReverse2) {

        tapeInnerModel(x, xNorm, eqNorm);

        const size_t m = _funInner->Range();
        const size_t n = _funInner->Domain();

        /**
         * Create the dynamic library model
         */

        auto cSourceInner = prepareInnerModelCompilation(jacInner, hessInner);

        /**
         * Second compiled model
         */
        // independent variables
        std::vector<ADCGD> ax(n);
        for (size_t j = 0; j < n; j++)
            ax[j] = x[j];

        CppAD::Independent(ax);

        CGAtomicFunBridge<double> atomicfun(_modelName, *_funInner, true);
        if (!jacInner.empty()) {
            atomicfun.setCustomSparseJacobianElements(jacInner);
        }
        if (!hessInner.empty()) {
            atomicfun.setCustomSparseHessianElements(hessInner);
        }

        std::vector<ADCGD> ay(m);
        atomicfun(ax, ay);
        std::vector<ADCGD> az = modelOuter(ay);

        // create f: ax -> az
        ADFun<CGD> fun2;
        fun2.Dependent(az);

        ModelCSourceGen<double> cSourceOuter(fun2, _modelName + "_outer");

        cSourceOuter.setCreateForwardZero(true);
        cSourceOuter.setCreateForwardOne(true);
        cSourceOuter.setCreateReverseOne(true);
        cSourceOuter.setCreateReverseTwo(createOuterReverse2);
        cSourceOuter.setCreateJacobian(true);
        cSourceOuter.setCreateHessian(true);
        cSourceOuter.setCreateSparseJacobian(true);
        cSourceOuter.setCreateSparseHessian(true);
        if (!jacOuter.empty()) {
            cSourceOuter.setCustomSparseJacobianElements(jacOuter);
        }
        if (!hessOuter.empty()) {
            cSourceOuter.setCustomSparseHessianElements(hessOuter);
        }

        /**
         * Create the dynamic library
         * (generate and compile source code)
         */
        ModelLibraryCSourceGen<double> compDynHelp(*cSourceInner, cSourceOuter);
        compDynHelp.setVerbose(this->verbose_);

        std::string folder = std::string("sources_atomiclibmodelbridge_") + (createOuterReverse2 ? "rev2_" : "dir_") + _modelName;

        DynamicModelLibraryProcessor<double> p(compDynHelp);

        GccCompiler<double> compiler;
        prepareTestCompilerFlags(compiler);
        compiler.setSourcesFolder(folder);
        compiler.setSaveToDiskFirst(true);
        _dynamicLib = p.createDynamicLibrary(compiler);

        /**
         * tape the model without atomics
         */
        tapeOuterModel(x, xNorm, eqNorm);
    }

protected:
    void tapeInnerModel(const CppAD::vector<Base>& x,
                        const CppAD::vector<Base>& xNorm,
                        const CppAD::vector<Base>& eqNorm) {
        const size_t n = x.size();

        // independent variables
        std::vector<ADCGD> ax(n);
        for (size_t j = 0; j < n; j++)
            ax[j] = x[j];

        CppAD::Independent(ax);
        if (xNorm.size() > 0) {
            for (size_t j = 0; j < n; j++)
                ax[j] *= xNorm[j];
        }

        /**
         * create the CppAD tape as usual
         */
        std::vector<ADCGD> ay = model(ax);

        if (eqNorm.size() > 0) {
            ASSERT_EQ(ay.size(), eqNorm.size());
            for (size_t i = 0; i < ay.size(); i++)
                ay[i] /= eqNorm[i];
        }

        // create f: U -> ay and vectors used for derivative calculations
        _funInner.reset(new ADFun<CGD>());
        _funInner->Dependent(ay);
    }

    void tapeOuterModel(const CppAD::vector<Base>& x,
                        const CppAD::vector<Base>& xNorm,
                        const CppAD::vector<Base>& eqNorm) {
        const size_t n = x.size();

        // independent variables
        std::vector<ADCGD> ax(n);
        for (size_t j = 0; j < n; j++)
            ax[j] = x[j];

        CppAD::Independent(ax);
        for (size_t j = 0; j < n; j++)
            ax[j] *= xNorm[j];

        /**
         * create the CppAD tape as usual
         */
        std::vector<ADCGD> ay = model(ax);
        if (eqNorm.size() > 0) {
            ASSERT_EQ(ay.size(), eqNorm.size());
            for (size_t i = 0; i < ay.size(); i++)
                ay[i] /= eqNorm[i];
        }

        std::vector<ADCGD> az = modelOuter(ay);

        // create f: ax -> az
        _funOuter.reset(new ADFun<CGD>());
        _funOuter->Dependent(az);
    }

    template<class Atomic>
    void tapeOuterModelWithAtomic(Atomic atomic,
                                  const CppAD::vector<Base> &x,
                                  const CppAD::vector<Base> &xNorm,
                                  const CppAD::vector<Base> &eqNorm) {
        const size_t n = _modelLib->Domain();
        const size_t m = _modelLib->Range();

        // independent variables
        std::vector<ADCGD> ax(n);
        for (size_t j = 0; j < n; j++)
            ax[j] = x[j];

        CppAD::Independent(ax);
        if (xNorm.size() > 0) {
            for (size_t j = 0; j < n; j++)
                ax[j] *= xNorm[j];
        }

        /**
         * create the CppAD tape as usual
         */
        std::vector<ADCGD> ay(m);

        atomic(ax, ay); // atomic function

        if (eqNorm.size() > 0) {
            ASSERT_EQ(ay.size(), eqNorm.size());
            for (size_t i = 0; i < ay.size(); i++)
                ay[i] /= eqNorm[i];
        }

        std::vector<ADCGD> az = modelOuter(ay);

        // create f: ax -> az
        _funOuterAtom.reset(new ADFun<CGD>());
        _funOuterAtom->Dependent(az);
    }

    std::unique_ptr<ModelCSourceGen<double>>
    prepareInnerModelCompilation(const std::vector<std::set<size_t> >& jacInner = {},
                                 const std::vector<std::set<size_t> >& hessInner = {}) {
        /**
         * Create the dynamic library model for the inner model
         */
        std::unique_ptr<ModelCSourceGen<double>> cSourceInner(new ModelCSourceGen<double>(*_funInner, _modelName));
        cSourceInner->setCreateForwardZero(true);
        cSourceInner->setCreateForwardOne(true);
        cSourceInner->setCreateReverseOne(true);
        cSourceInner->setCreateReverseTwo(true);
        cSourceInner->setCreateJacobian(true);
        cSourceInner->setCreateHessian(true);
        cSourceInner->setCreateSparseJacobian(true);
        cSourceInner->setCreateSparseHessian(true);

        if (!jacInner.empty()) {
            cSourceInner->setCustomSparseJacobianElements(jacInner);
        }
        if (!hessInner.empty()) {
            cSourceInner->setCustomSparseHessianElements(hessInner);
        }

        return cSourceInner;
    }

    std::unique_ptr<GenericModel<double>> compileInnerModel(const std::string& folderNamePrefix,
                                                            const std::vector<std::set<size_t> >& jacInner = {},
                                                            const std::vector<std::set<size_t> >& hessInner = {}) {

        auto cSourceInner = prepareInnerModelCompilation(jacInner, hessInner);
        /**
         * Create the dynamic library
         * (generate and compile source code)
         */
        ModelLibraryCSourceGen<double> libSrcGen(*cSourceInner);
        libSrcGen.setVerbose(this->verbose_);

        std::string folder = std::string("sources_inner_model") + folderNamePrefix + _modelName;

        DynamicModelLibraryProcessor<double> p(libSrcGen);

        GccCompiler<double> compiler;
        prepareTestCompilerFlags(compiler);
        compiler.setSourcesFolder(folder);
        compiler.setSaveToDiskFirst(true);
        _dynamicLib = p.createDynamicLibrary(compiler);

        return _dynamicLib->model(_modelName);
    }

};

} // END cg namespace
} // END CppAD namespace

#endif
