#ifndef CPPAD_CG_TEST_CG_ATOMIC_GENERIC_MODEL_INCLUDED
#define CPPAD_CG_TEST_CG_ATOMIC_GENERIC_MODEL_INCLUDED
/* --------------------------------------------------------------------------
 *  CppADCodeGen: C++ Algorithmic Differentiation with Source Code Generation:
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

#include "CppADCGDynamicAtomicTest.hpp"

namespace CppAD {
namespace cg {

class CGAtomicGenericModelTest : public CppAD::cg::CppADCGDynamicAtomicTest {
public:
    using Super = CppADCGDynamicAtomicTest;
    using Base = Super::Base;
    using CGD = Super::CGD;
    using ADCGD = Super::ADCGD;
    using sparsity_pattern = CppAD::sparse_rc<std::vector<size_t>>;
    using sparse_matrix = CppAD::sparse_rcv<std::vector<size_t>, std::vector<Base>>;
protected:
    std::unique_ptr<ADFun<Base>> _funBaseOuterAtom; // outer model tape with atomics
    std::unique_ptr<GenericModel<double>> _innerModel;
    std::unique_ptr<CGAtomicGenericModel<double>> _cgAtomic;
    const CppAD::vector<Base> _x;
    const CppAD::vector<Base> _xNorm = {};
    const CppAD::vector<Base> _eqNorm = {};
    const std::vector<std::set<size_t> > _jacSpar;
    const std::vector<std::set<size_t> > _hessSpar;
    std::vector<Base> _stdX;
    std::vector<Base> _tx2;
    size_t _m;
    size_t _n;
public:

    CGAtomicGenericModelTest(std::string modelName,
                             const size_t n,
                             std::vector<std::set<size_t> > jacSpar = {},
                             std::vector<std::set<size_t> > hessSpar = {},
                             bool verbose = false,
                             bool printValues = true) :
            CGAtomicGenericModelTest(std::move(modelName),
                                     Super::makeValues(n), std::move(jacSpar), std::move(hessSpar),
                                     verbose, printValues) {
    }

    CGAtomicGenericModelTest(std::string modelName,
                             const CppAD::vector<Base>& x,
                             std::vector<std::set<size_t> > jacSpar = {},
                             std::vector<std::set<size_t> > hessSpar = {},
                             bool verbose = false,
                             bool printValues = true) :
            CppADCGDynamicAtomicTest(std::move(modelName), verbose, printValues),
            _x(x),
            _jacSpar(std::move(jacSpar)),
            _hessSpar(std::move(hessSpar)),
            _stdX(x.size()),
            _tx2(x.size() * 2),
            _m(0),
            _n(0) {
    }

    std::vector<ADCGD> model(const std::vector<ADCGD> &x) override = 0;

    virtual std::vector<AD<Base>> modelBaseOuter(const std::vector<AD<Base>> &y) {
        return y;
    }

    void TearDown() override {
        // need to clear memory here because CppAD checks if there are any memory still in use
        _funBaseOuterAtom.reset();
        _cgAtomic.reset();
        _innerModel.reset();
    }

    ~CGAtomicGenericModelTest() override = default;

protected:

    void SetUp() override {
        /**
        * Inner model
        */
        tapeInnerModel(_x, _xNorm, _eqNorm);

        _m = _funInner->Range();
        _n = _funInner->Domain();

        /**
        * Create the dynamic library model for the inner model
        */
        _innerModel = compileInnerModel("", _jacSpar, _hessSpar);

        _cgAtomic.reset(new CGAtomicGenericModel<double>(*_innerModel));

        /**
        * Outer model
        */
        tapeBaseOuterModelWithAtomic(*_cgAtomic, _x, _xNorm, _eqNorm);

        std::copy(_x.data(), _x.data() + _x.size(), _stdX.data());
        for (size_t i = 0; i < _n; ++i) _tx2[i * 2] = _stdX[i];
    }

    void testCGAtomicGenericModelForwardZero() {
        /**
         * Compute values using only the inner model
         */
        const auto yOrig = _innerModel->ForwardZero(_stdX);

        std::vector<Base> ty2(yOrig.size() * 2);
        for (size_t i = 0; i < _m; ++i) ty2[i * 2] = yOrig[i];

        /**
         * Outer
         */
        auto z = _funBaseOuterAtom->Forward(0, _stdX);

        /**
         * Compare
         */
        if (this->printValues_) {
            std::cout << "GenericModel:         ";
            print(yOrig);
            std::cout << std::endl;

            std::cout << "CGAtomicGenericModel: ";
            print(z);
            std::cout << std::endl;
        }

        ASSERT_TRUE(compareValues(z, yOrig));
    }

    void testCGAtomicGenericModelReverseOne() {
        const auto yOrig = _innerModel->ForwardZero(_stdX);

        // 1st order
        std::vector<Base> py1(_m, 0);
        std::vector<std::vector<Base>> px1(_m);
        for (size_t j = 0; j < _m; ++j) {
            py1[j] = 1;
            px1[j] = _innerModel->ReverseOne(_stdX, yOrig, py1);
            py1[j] = 0;
        }

        /**
         * Outer
         */
        auto z = _funBaseOuterAtom->Forward(0, _stdX);

        std::vector<std::vector<Base>> pxOuter(_m);
        for (size_t i = 0; i < _m; ++i) {
            py1[i] = 1;
            pxOuter[i] = _funBaseOuterAtom->Reverse(1, py1);
            py1[i] = 0;
        }

        /**
         * Compare
         */
        for (size_t i = 0; i < _m; ++i) {
            ASSERT_TRUE(compareValues(pxOuter[i], px1[i]));
        }
    }

    void testCGAtomicGenericModelReverseTwo() {
        const auto yOrig = _innerModel->ForwardZero(_stdX);

        std::vector<Base> ty2(yOrig.size() * 2);
        for (size_t i = 0; i < _m; ++i) ty2[i * 2] = yOrig[i];

        // 2nd order
        std::vector<Base> py2(2 * _m, 0);
        std::vector<std::vector<Base>> px2(_m);
        for (size_t j = 0; j < _n; ++j) _tx2[j * 2 + 1] = 1;

        for (size_t i = 0; i < _m; ++i) {
            py2[i * 2 + 1] = 1;
            px2[i] = _innerModel->ReverseTwo(_tx2, ty2, py2);
            py2[i * 2 + 1] = 0;
        }

        /**
         * Outer
         */
        std::vector<std::vector<Base>> px2Outer(_m);
        _funBaseOuterAtom->Forward(1, _tx2);
        for (size_t i = 0; i < _m; ++i) {
            py2[i * 2 + 1] = 1;
            px2Outer[i] = _funBaseOuterAtom->Reverse(2, py2);
            py2[i * 2 + 1] = 0;
        }

        /**
         * Compare
         */
        if (this->printValues_) {
            std::cout << "GenericModel rev 2:" << std::endl;
            print(px2);
            std::cout << std::endl;

            std::cout << "CGAtomicGenericModel rev 2:" << std::endl;
            print(px2Outer);
            std::cout << std::endl;
        }

        for (size_t i = 0; i < _m; ++i) {
            ASSERT_TRUE(compareValues(px2Outer[i], px2[i]));
        }
    }

    void testCGAtomicGenericModelJacobian() {
        std::vector<Base> jacOrig;
        std::vector<size_t> jacRow, jacCol;
        _innerModel->SparseJacobian(_stdX, jacOrig, jacRow, jacCol);

        /**
         * Outer
         */
        std::vector<std::set<size_t> > localJacSpar;
        std::vector<Base> jacOuter(jacRow.size());
        sparse_jacobian_work jacWork;

        if (localJacSpar.empty()) {
            localJacSpar = jacobianSparsitySet<std::vector<std::set<size_t> > >(*_funBaseOuterAtom);
        }

        _funBaseOuterAtom->SparseJacobianReverse(_stdX, localJacSpar, jacRow, jacCol, jacOuter, jacWork);

        /**
         * Compare
         */
        if (this->printValues_) {
            std::cout << "GenericModel sparse Jacobian:" << std::endl;
            printTripletMatrix(jacRow, jacCol, jacOrig);

            std::cout << "CGAtomicGenericModel sparse Jacobian:" << std::endl;
            printTripletMatrix(jacRow, jacCol, jacOuter);
        }

        ASSERT_TRUE(compareValues(jacOuter, jacOrig));
    }

    void testCGAtomicGenericModelHessian() {
        std::vector<Base> w(_m, 1.0);
        std::vector<Base> hessOrig;
        std::vector<size_t> hessRows;
        std::vector<size_t> hessCols;
        _innerModel->SparseHessian(_stdX, w, hessOrig, hessRows, hessCols);

        /**
         * Outer
         */
        std::vector<std::set<size_t> > localHessSpar = _hessSpar;
        if (localHessSpar.empty()) {
            localHessSpar = hessianSparsitySet<std::vector<std::set<size_t> > >(*_funBaseOuterAtom);
        }

        sparsity_pattern hessPattern = toSparsityPattern<std::vector<size_t>>(localHessSpar, _n, _n);
        sparsity_pattern subHess = _hessSpar.empty() ? hessPattern :
                                   toSparsityPattern<std::vector<size_t>>(_hessSpar, _n, _n);
        sparse_matrix subHessMatrix(subHess);
        sparse_hes_work work;
        _funBaseOuterAtom->sparse_hes(_stdX, w, subHessMatrix, hessPattern, "cppad.general", work);

        /**
         * Compare
         */
        if (this->printValues_) {
            std::cout << "GenericModel sparse hessian:" << std::endl;
            printTripletMatrix(hessRows, hessCols, hessOrig);

            std::cout << "CGAtomicGenericModel sparse hessian:" << std::endl;
            printTripletMatrix(subHessMatrix.row(), subHessMatrix.col(), subHessMatrix.val());
        }

        ASSERT_TRUE(compareValues(subHessMatrix.val(), hessOrig));
    }

private:

    template<class Atomic>
    void tapeBaseOuterModelWithAtomic(Atomic &atomic,
                                      const CppAD::vector<Base> &x,
                                      const CppAD::vector<Base> &xNorm,
                                      const CppAD::vector<Base> &eqNorm) {
        const size_t n = _funInner->Domain();
        const size_t m = _funInner->Range();

        // independent variables
        std::vector<AD<Base>> ax(n);
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
        std::vector<AD<Base>> ay(m);

        atomic(ax, ay); // atomic function

        if (eqNorm.size() > 0) {
            ASSERT_EQ(ay.size(), eqNorm.size());
            for (size_t i = 0; i < ay.size(); i++)
                ay[i] /= eqNorm[i];
        }

        std::vector<AD<Base>> az = modelBaseOuter(ay);

        // create f: ax -> az
        _funBaseOuterAtom.reset(new ADFun<Base>());
        _funBaseOuterAtom->Dependent(az);
    }

};

} // END cg namespace
} // END CppAD namespace

#endif
