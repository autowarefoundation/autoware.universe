#ifndef CPPAD_CG_TEST_CPPADCGMODELTEST_INCLUDED
#define CPPAD_CG_TEST_CPPADCGMODELTEST_INCLUDED
/* --------------------------------------------------------------------------
 *  CppADCodeGen: C++ Algorithmic Differentiation with Source Code Generation:
 *    Copyright (C) 2019 Joao Leal
 *    Copyright (C) 2012 Ciengis
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

class CppADCGModelTest : public CppADCGTest {
public:
    using CGD = CG<double>;
    using ADCG = AD<CGD>;
public:

    explicit CppADCGModelTest(bool verbose = false,
                              bool printValues = false) :
            CppADCGTest(verbose, printValues) {
    }

    static std::vector<CGD> makeVector(const std::vector<Base>& x) {
        std::vector<CGD> x2(x.size());
        for (size_t i = 0; i < x.size(); i++) {
            x2[i] = x[i];
        }
        return x2;
    }

    /**
     * Compares the results from forward zero.
     *
     * @param model
     * @param fun
     * @param x independent vector values
     * @param epsilonR relative error
     * @param epsilonA absolute error
     */
    static void testForwardZeroResults(GenericModel<Base>& model,
                                       ADFun<CGD>& fun,
                                       ADFun<Base>* fun2,
                                       const std::vector<Base>& x,
                                       double epsilonR = 1e-14,
                                       double epsilonA = 1e-14) {
        ASSERT_EQ(model.Domain(), fun.Domain());
        ASSERT_EQ(model.Range(), fun.Range());

        std::vector<CGD> x2 = makeVector(x);

        std::vector<CGD> dep = fun.Forward(0, x2);
        std::vector<Base> depCGen = model.ForwardZero(x);

        ASSERT_TRUE(compareValues(depCGen, dep, epsilonR, epsilonA));

        if (fun2 != nullptr) {
            std::vector<Base> yOuter = fun2->Forward(0, x);
            ASSERT_TRUE(compareValues<Base>(yOuter, depCGen, epsilonR, epsilonA));
        }
    }

    static void testForwardOneResults(GenericModel<Base>& model,
                                      ADFun<CGD>& fun,
                                      ADFun<Base>* funWrapModel,
                                      const std::vector<Base>& x,
                                      double epsilonR = 1e-14,
                                      double epsilonA = 1e-14) {
        ASSERT_EQ(model.Domain(), fun.Domain());
        ASSERT_EQ(model.Range(), fun.Range());

        size_t n = model.Domain();

        /**
         * Test first order forward mode
         */
        size_t k = 1;
        size_t k1 = k + 1;

        std::vector<Base> x_p(n, 0.0);
        std::vector<CGD> x_pOrig(n);
        std::vector<Base> tx(k1 * n);
        for (size_t j = 0; j < n; j++)
            tx[j * k1] = x[j]; // zero order
        for (size_t j = 0; j < n; j++)
            tx[j * k1 + 1] = 0; // first order

        for (size_t j = 0; j < n; j++) {
            x_p[j] = 1;
            x_pOrig[j] = 1;
            tx[j * k1 + 1] = 1;

            std::vector<CGD> y_pOrig = fun.Forward(1, x_pOrig);
            std::vector<Base> y_pInner = model.ForwardOne(tx);

            ASSERT_TRUE(compareValues<Base>(y_pInner, y_pOrig, epsilonR, epsilonA));

            if (funWrapModel != nullptr) {
                std::vector<Base> y_pOuter = funWrapModel->Forward(1, x_p);
                ASSERT_TRUE(compareValues<Base>(y_pOuter, y_pOrig, epsilonR, epsilonA));
            }

            x_p[j] = 0;
            x_pOrig[j] = 0;
            tx[j * k1 + 1] = 0;
        }
    }

    static void testReverseOneResults(GenericModel<Base>& model,
                                      ADFun<CGD>& fun,
                                      ADFun<Base>* funWrapModel,
                                      const std::vector<Base>& x,
                                      double epsilonR = 1e-14,
                                      double epsilonA = 1e-14) {
        ASSERT_EQ(model.Domain(), fun.Domain());
        ASSERT_EQ(model.Range(), fun.Range());

        size_t m = model.Range();
        size_t n = model.Domain();

        size_t k = 0;
        size_t k1 = k + 1;

        std::vector<Base> yInner = model.ForwardZero(x);

        std::vector<Base> w(m, 0.0);
        std::vector<CGD> wOrig(m);
        std::vector<Base> tx(k1 * n);
        for (size_t j = 0; j < n; j++)
            tx[j * k1] = x[j]; // zero order
        std::vector<Base> ty(k1 * m);
        for (size_t i = 0; i < m; i++)
            ty[i * k1] = yInner[i]; // zero order

        for (size_t i = 0; i < m; i++) {
            w[i] = 1;
            wOrig[i] = 1;

            std::vector<CGD> dwOrig = fun.Reverse(1, wOrig);
            std::vector<Base> dwInner = model.ReverseOne(tx, ty, w);

            ASSERT_TRUE(compareValues<Base>(dwInner, dwOrig, epsilonR, epsilonA));

            if (funWrapModel != nullptr) {
                std::vector<Base> dwOuter = funWrapModel->Reverse(1, w);
                ASSERT_TRUE(compareValues<Base>(dwOuter, dwOrig, epsilonR, epsilonA));
            }

            w[i] = 0;
            wOrig[i] = 0;
        }
    }

    static void testReverseTwoResults(GenericModel<Base>& model,
                                      ADFun<CGD>& fun,
                                      ADFun<Base>* funWrapModel,
                                      const std::vector<Base>& x,
                                      double epsilonR = 1e-14,
                                      double epsilonA = 1e-14) {
        ASSERT_EQ(model.Domain(), fun.Domain());
        ASSERT_EQ(model.Range(), fun.Range());

        size_t m = model.Range();
        size_t n = model.Domain();

        std::vector<Base> yInner = model.ForwardZero(x);

        size_t k = 1;
        size_t k1 = k + 1;
        std::vector<Base> tx(k1 * n);
        std::vector<Base> ty(k1 * m);
        std::vector<Base> py(k1 * m);
        std::vector<CGD> pyOrig(k1 * m);
        //wOrig.resize(k1 * m);
        for (size_t j = 0; j < n; j++) {
            tx[j * k1] = x[j]; // zero order
            tx[j * k1 + 1] = 0; // first order
        }
        for (size_t i = 0; i < m; i++) {
            ty[i * k1] = yInner[i]; // zero order
            py[i * k1] = 0.0;
            py[i * k1 + 1] = 1.0; // first order
            pyOrig[i * k1] = 0.0;
            pyOrig[i * k1 + 1] = 1.0; // first order
        }

        std::vector<Base> x_p(n, 0.0);
        std::vector<CGD> x_pOrig(n);

        for (size_t j = 0; j < n; j++) {
            x_p[j] = 1;
            x_pOrig[j] = 1;
            tx[j * k1 + 1] = 1;

            fun.Forward(1, x_pOrig);
            std::vector<CGD> dwOrig = fun.Reverse(2, pyOrig);
            std::vector<Base> dwInner = model.ReverseTwo(tx, ty, py);

            // only compare second order information
            // (location of the elements is different then if py.size() == m)
            ASSERT_EQ(dwOrig.size(), n * k1);
            ASSERT_EQ(dwOrig.size(), dwInner.size());

            for (size_t j2 = 0; j2 < n; j2++) {
                ASSERT_TRUE(nearEqual(dwInner[j2 * k1], dwOrig[j2 * k1].getValue()));
            }

            if(funWrapModel != nullptr) {
                funWrapModel->Forward(1, x_p);
                std::vector<Base> dwOuter = funWrapModel->Reverse(2, py);
                ASSERT_EQ(dwOrig.size(), dwOuter.size());
                for (size_t j2 = 0; j2 < n; j2++) {
                    ASSERT_TRUE(nearEqual(dwOuter[j2 * k1], dwOrig[j2 * k1].getValue()));
                }
            }

            x_p[j] = 0;
            x_pOrig[j] = 0;
            tx[j * k1 + 1] = 0;
        }
    }

    /**
     * Compares the results from a dense Jacobian.
     *
     * @param model
     * @param fun
     * @param x independent vector values
     * @param epsilonR relative error
     * @param epsilonA absolute error
     */
    void testDenseJacResults(GenericModel<Base>& model,
                             ADFun<CGD>& fun,
                             const std::vector<Base>& x,
                             double epsilonR = 1e-14,
                             double epsilonA = 1e-14) {
        ASSERT_EQ(model.Domain(), fun.Domain());
        ASSERT_EQ(model.Range(), fun.Range());

        std::vector<CGD> x2 = makeVector(x);

        auto jac = fun.Jacobian(x2);
        if (verbose_) {
            std::cout << "ADFun Jacobian" << std::endl;
            print(jac);
        }

        auto depCGen = model.Jacobian(x);

        ASSERT_TRUE(compareValues(depCGen, jac, epsilonR, epsilonA));
    }

    /**
     * Compares the results from a Dense Hessian.
     *
     * @param model
     * @param fun
     * @param x independent vector values
     * @param epsilonR relative error
     * @param epsilonA absolute error
     */
    void testDenseHessianResults(GenericModel<Base>& model,
                                 ADFun<CGD>& fun,
                                 const std::vector<Base>& x,
                                 double epsilonR = 1e-14,
                                 double epsilonA = 1e-14) {
        // dimensions
        ASSERT_EQ(model.Domain(), fun.Domain());
        ASSERT_EQ(model.Range(), fun.Range());

        std::vector<CGD> x2 = makeVector(x);
        std::vector<CGD> w2(fun.Range(), 1.0);
        std::vector<Base> w(fun.Range(), 1.0);

        std::vector<CGD> hess = fun.Hessian(x2, w2);

        if (verbose_) {
            std::cout << "ADFun Hessian" << std::endl;
            print(hess);
        }

        auto depCGen = model.Hessian(x, w);

        ASSERT_TRUE(compareValues(depCGen, hess, epsilonR, epsilonA));
    }

private:
    void testJacobianResults(GenericModel<Base>& model,
                             ADFun<Base>* funWrapModel,
                             const std::vector<CGD>& jac,
                             const std::vector<Base>& x,
                             bool customSparsity,
                             double epsilonR = 1e-14,
                             double epsilonA = 1e-14) {
        std::vector<Base> jacCGen;
        std::vector<size_t> row, col;

        model.SparseJacobian(x, jacCGen, row, col);

        std::vector<Base> jacCGenDense(jac.size());

        for (size_t i = 0; i < jacCGen.size(); i++) {
            size_t p = row[i] * x.size() + col[i];
            jacCGenDense[p] = jacCGen[i];
        }

        std::vector<CGD> jacAdFunPartial(jac.size());
        if (customSparsity) {
            for (size_t i = 0; i < jacCGen.size(); i++) {
                size_t p = row[i] * x.size() + col[i];
                jacAdFunPartial[p] = jac[p];
            }
        } else {
            jacAdFunPartial = jac;
        }

        if (verbose_) {
            std::cout << "sparse Jacobian (in a dense format)" << std::endl;
            print(jacCGenDense);
        }

        ASSERT_TRUE(this->compareValues(jacCGenDense, jacAdFunPartial, epsilonR, epsilonA));

        if (funWrapModel != nullptr) {
            auto jacOuter = funWrapModel->SparseJacobian(x);

            if (verbose_) {
                std::cout << "ADFun2 Jacobian" << std::endl;
                print(jacOuter);
            }

            ASSERT_TRUE(compareValues<Base>(jacOuter, jac, epsilonR, epsilonA));

            // sparse reverse
            const auto jacSparsityWrap = jacobianReverseSparsitySet<std::vector<std::set<size_t>>, Base>(*funWrapModel);

            sparse_jacobian_work workWrapFun;
            std::vector<Base> jacWrapFunSparse(row.size());
            funWrapModel->SparseJacobianReverse(x, jacSparsityWrap, row, col, jacWrapFunSparse, workWrapFun);

            ASSERT_TRUE(compareValues<Base>(jacCGen, jacWrapFunSparse, epsilonR, epsilonA));
        }
    }

    void testHessianResults(GenericModel<Base>& model,
                            ADFun<Base>* funWrapModel,
                            const std::vector<CGD>& hess,
                            const std::vector<Base>& x,
                            bool customSparsity,
                            double epsilonR = 1e-14,
                            double epsilonA = 1e-14) {
        std::vector<CGD> w2(model.Range(), 1.0);
        std::vector<Base> w(model.Range(), 1.0);
        std::vector<size_t> row, col;
        std::vector<Base> hessCGen;

        model.SparseHessian(x, w, hessCGen, row, col);

        std::vector<Base> hessCGenDense(hess.size());
        for (size_t i = 0; i < hessCGen.size(); i++) {
            size_t p = row[i] * x.size() + col[i];
            hessCGenDense[p] = hessCGen[i];
        }

        std::vector<CGD> hessAdFunPartial(hess.size());
        if (customSparsity) {
            for (size_t i = 0; i < hessCGen.size(); i++) {
                size_t p = row[i] * x.size() + col[i];
                hessAdFunPartial[p] = hess[p];
            }
        } else {
            hessAdFunPartial = hess;
        }

        if (verbose_) {
            std::cout << "sparse Hessian (in a dense format)" << std::endl;
            print(hessCGenDense);
        }

        ASSERT_TRUE(this->compareValues(hessCGenDense, hessAdFunPartial, epsilonR, epsilonA));

        if(funWrapModel != nullptr) {
            auto hessFunWrap = funWrapModel->SparseHessian(x, w);

            if (verbose_) {
                std::cout << "ADFun2 Hessian" << std::endl;
                print(hessFunWrap);
            }

            ASSERT_TRUE(compareValues<Base>(hessFunWrap, hessAdFunPartial, epsilonR, epsilonA));

            const auto hessSparsityWrap = hessianSparsitySet<std::vector<std::set<size_t>>, Base>(*funWrapModel);

            sparse_hessian_work workWrapFun;
            std::vector<Base> hessWrapFunSparse(row.size());
            funWrapModel->SparseHessian(x, w, hessSparsityWrap, row, col, hessWrapFunSparse, workWrapFun);

            ASSERT_TRUE(compareValues<double>(hessCGen, hessWrapFunSparse, epsilonR, epsilonA));
        }
    }

public:

    static void testJacobianSparsity(ADFun<CGD>& fun1,
                                     ADFun<Base>& fun2) {
        const std::vector<bool> jacSparsityOrig = jacobianForwardSparsity<std::vector<bool>, CGD>(fun1);
        const std::vector<bool> jacSparsityOuter = jacobianForwardSparsity<std::vector<bool>, double>(fun2);

        compareBoolValues(jacSparsityOrig, jacSparsityOuter);

        const std::vector<bool> jacSparsityOrigRev = jacobianReverseSparsity<std::vector<bool>, CGD>(fun1);
        const std::vector<bool> jacSparsityOuterRev = jacobianReverseSparsity<std::vector<bool>, double>(fun2);

        compareBoolValues(jacSparsityOrigRev, jacSparsityOrig);
        compareBoolValues(jacSparsityOrigRev, jacSparsityOuterRev);
    }

    /**
     * Compares the results from a sparse Jacobian.
     *
     * @param n_tests number of times to run this test
     * @param model
     * @param fun
     * @param x independent vector values
     * @param customSparsity
     * @param epsilonR relative error
     * @param epsilonA absolute error
     */
    void testSparseJacobianResults(size_t n_tests,
                                   GenericModel<Base>& model,
                                   ADFun<CGD>& fun,
                                   ADFun<Base>* funWrapModel,
                                   const std::vector<Base>& x,
                                   bool customSparsity,
                                   double epsilonR = 1e-14,
                                   double epsilonA = 1e-14) {
        ASSERT_EQ(model.Domain(), fun.Domain());
        ASSERT_EQ(model.Range(), fun.Range());

        std::vector<CGD> x2 = makeVector(x);
        std::vector<CGD> jac = fun.Jacobian(x2);

        if (verbose_) {
            std::cout << "ADFun Jacobian" << std::endl;
            print(jac);
        }

        for (size_t i = 0; i< n_tests; ++i) {
            testJacobianResults(model, funWrapModel, jac, x, customSparsity, epsilonR, epsilonA);
        }
    }

    /**
     * Compares the results from a sparse Hessian.
     *
     * @param model
     * @param fun
     * @param x independent vector values
     * @param epsilonR relative error
     * @param epsilonA absolute error
     */
    void testSparseHessianResults(size_t n_tests,
                                  GenericModel<Base>& model,
                                  ADFun<CGD>& fun,
                                  ADFun<Base>* funWrapModel,
                                  const std::vector<Base>& x,
                                  bool customSparsity,
                                  double epsilonR = 1e-14,
                                  double epsilonA = 1e-14) {
        ASSERT_EQ(model.Domain(), fun.Domain());
        ASSERT_EQ(model.Range(), fun.Range());

        std::vector<CGD> x2 = makeVector(x);
        std::vector<CGD> w2(fun.Range(), 1.0);
        std::vector<Base> w(fun.Range(), 1.0);

        std::vector<CGD> hess = fun.Hessian(x2, w2);

        if (verbose_) {
            std::cout << "ADFun Hessian" << std::endl;
            print(hess);
        }

        for (size_t i = 0; i < n_tests; ++i) {
            testHessianResults(model, funWrapModel, hess, x, customSparsity, epsilonR, epsilonA);
        }
    }

};

} // END cg namespace
} // END CppAD namespace

#endif