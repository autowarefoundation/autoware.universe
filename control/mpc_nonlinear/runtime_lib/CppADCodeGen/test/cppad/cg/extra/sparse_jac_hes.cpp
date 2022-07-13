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

#include <vector>
#include <valarray>

#include <cppad/cg/cppadcg.hpp>
#include <gtest/gtest.h>
#include "CppADCGTest.hpp"

using namespace CppAD;
using namespace CppAD::cg;
using namespace std;

class SparseJacHes : public CppADCGTest {
public:
    using std_vector_set = std::vector<std::set<size_t> > ;
    using cppad_vector_set = CppAD::vector<std::set<size_t> >;
public:

    inline static std::vector<AD<double> > model1(const std::vector<AD<double> >& X) {
        std::vector<AD<double> > Y(2);

        Y[0] = X[0] * X[1] * X[2] + X[0];
        Y[1] = X[1] * X[2] + 10;

        return Y;
    }

    inline static std::vector<AD<double> > model2(const std::vector<AD<double> >& X) {
        std::vector<AD<double> > Y(1);

        Y[0] = X[1] / X[0] * X[2];

        return Y;
    }

    void multiTestModel(size_t n,
                        std::vector<AD<double> > (*model)(const std::vector<AD<double> >& X),
                        const std::vector<size_t>& jacRow, const std::vector<size_t>& jacCol,
                        const std::vector<size_t>& hesRow, const std::vector<size_t>& hesCol) {
        testModel<CppAD::vector<double>, std_vector_set, std::vector<size_t> >(n, model, jacRow, jacCol, hesRow, hesCol);
        testModel<std::valarray<double>, std_vector_set, std::vector<size_t> >(n, model, jacRow, jacCol, hesRow, hesCol);
        testModel<std::vector<double>, cppad_vector_set, std::vector<size_t> >(n, model, jacRow, jacCol, hesRow, hesCol);
        testModel<CppAD::vector<double>, cppad_vector_set, std::vector<size_t> >(n, model, jacRow, jacCol, hesRow, hesCol);
    }

    template <class VectorBase, class VectorSet, class VectorSize>
    void testModel(size_t n,
                   std::vector<AD<double> > (*model)(const std::vector<AD<double> >& X),
                   const std::vector<size_t>& jacRow, const std::vector<size_t>& jacCol,
                   const std::vector<size_t>& hesRow, const std::vector<size_t>& hesCol) {

        size_t i, j;

        // domain space vector
        std::vector<AD<double> > X(n);
        for (i = 0; i < n; i++)
            X[i] = AD<double> (i + 1);

        // declare independent variables and starting recording
        Independent(X);

        std::vector<AD<double> > Y = model(X);
        size_t m = Y.size();

        // create f: x -> y and stop tape recording
        ADFun<double> f(X, Y);

        // new value for the independent variable vector
        VectorBase x(n);
        for (i = 0; i < n; i++)
            x[i] = 3 * double(i + 1);

        // second derivative of y[1]
        VectorBase w(m);
        for (size_t i = 0; i < m; i++)
            w[i] = i + 1;

        /**
         * zero-order
         */
        VectorBase check_y = f.Forward(0, x);

        /**
         * Jacobian
         */
        // sparsity pattern
        VectorSet s(m);
        for (i = 0; i < m; i++)
            s[i].insert(i);
        VectorSet jsparsity = f.RevSparseJac(m, s);

        // evaluate
        size_t jnnz = jacRow.size();
        VectorSize jrow(jnnz), jcol(jnnz);
        for (size_t jj = 0; jj < jnnz; jj++) {
            jrow[jj] = jacRow[jj];
            jcol[jj] = jacCol[jj];
        }
        VectorBase check_jac(jnnz);
        sparse_jacobian_work jwork;

        size_t n_sweep_j = f.SparseJacobianForward(x, jsparsity, jrow, jcol, check_jac, jwork);

        /**
         * Hessian
         */
        // determine the sparsity pattern p for Hessian of w^T F
        VectorSet r(n);
        for (j = 0; j < n; j++)
            r[j].insert(j);
        f.ForSparseJac(n, r);
        //
        s.resize(1);
        for (i = 0; i < m; i++)
            if (w[i] != 0)
                s[0].insert(i);
        VectorSet hsparsity = f.RevSparseHes(n, s);

        // evaluate
        size_t hnnz = hesRow.size();
        VectorSize hrow(hnnz), hcol(hnnz);
        for (size_t h = 0; h < hnnz; h++) {
            hrow[h] = hesRow[h];
            hcol[h] = hesCol[h];
        }
        VectorBase check_hes(hnnz);
        sparse_hessian_work hwork;

        size_t n_sweep_h = f.SparseHessian(x, w, hsparsity, hrow, hcol, check_hes, hwork);

        size_t n_sweep = 1 + n_sweep_j + 2 * n_sweep_h;

        /**
         * Jacobian + Hessian
         */
        VectorBase y(m);
        VectorBase jac(jnnz);
        VectorBase hes(hnnz);

        SparseForjacHessianWork work;
        size_t n_sweep_jh = 1 + sparseForJacHessian(f, x, w,
                                                    y,
                                                    jsparsity, jrow, jcol, jac,
                                                    hsparsity, hrow, hcol, hes,
                                                    work);
        if (this->verbose_) {
            std::cout << "n_sweep_jh = " << n_sweep_jh
                    << "  n_sweep = " << n_sweep << std::endl;
        }

        ASSERT_TRUE(compareValues<double>(y, check_y));
        ASSERT_TRUE(compareValues<double>(jac, check_jac));
        ASSERT_TRUE(compareValues<double>(hes, check_hes));

    }

};

TEST_F(SparseJacHes, model1) {
    std::vector<size_t> jrow(2), jcol(2);
    jrow[0] = 0;
    jcol[0] = 2;
    jrow[1] = 1;
    jcol[1] = 1;

    std::vector<size_t> hrow(5), hcol(5);
    hrow[0] = 0;
    hcol[0] = 1;
    hrow[1] = 0;
    hcol[1] = 2;
    hrow[2] = 1;
    hcol[2] = 0;
    hrow[3] = 1;
    hcol[3] = 2;
    hrow[4] = 2;
    hcol[4] = 1;

    multiTestModel(3, model1,
                   jrow, jcol,
                   hrow, hcol);
}

TEST_F(SparseJacHes, model2) {
    std::vector<size_t> jrow(1), jcol(1);
    jrow[0] = 0;
    jcol[0] = 2;

    std::vector<size_t> hrow(4), hcol(4);
    hrow[0] = 0;
    hrow[1] = 0;
    hrow[2] = 0;
    hrow[3] = 1;

    hcol[0] = 0;
    hcol[1] = 1;
    hcol[2] = 2;
    hcol[3] = 2;

    multiTestModel(3, model2,
                   jrow, jcol,
                   hrow, hcol);
}
