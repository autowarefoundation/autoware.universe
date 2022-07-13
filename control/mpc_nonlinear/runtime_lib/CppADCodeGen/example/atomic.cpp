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

#include <cppad/cg.hpp>

using namespace CppAD;
using namespace CppAD::cg;

using CGD = CG<double>;
using ADCG = AD<CGD>;

class CustomAtomicExample : public atomic_base<double> {
public:
    CustomAtomicExample() : atomic_base<double>("atomic", set_sparsity_enum) {}

    bool forward(size_t p,
                 size_t q,
                 const CppAD::vector<bool>& vx,
                 CppAD::vector<bool>& vy,
                 const CppAD::vector<double>& tx,
                 CppAD::vector<double>& ty) override {
        size_t q1 = q + 1;
# ifndef NDEBUG
        size_t n = tx.size() / q1;
        size_t m = ty.size() / q1;
# endif
        assert(n == 3);
        assert(m == 2);
        assert(p <= q);

        // this example only implements up to second order forward mode
        bool ok = q <= 2;
        if (!ok)
            return ok;

        // check for defining variable information
        // This case must always be implemented
        if (vx.size() > 0) {
            vy[0] = vx[2];
            vy[1] = vx[0] || vx[1];
        }
        // ------------------------------------------------------------------
        // Zero forward mode.
        // This case must always be implemented
        // f(x) = [ x_2 * x_2 ]
        //        [ x_0 * x_1 ]
        // y^0  = f( x^0 )
        if (p <= 0) {   // y_0^0 = x_2^0 * x_2^0
            ty[0 * q1 + 0] = tx[2 * q1 + 0] * tx[2 * q1 + 0];
            // y_1^0 = x_0^0 * x_1^0
            ty[1 * q1 + 0] = tx[0 * q1 + 0] * tx[1 * q1 + 0];
        }

        if (q <= 0)
            return ok;
        // ------------------------------------------------------------------
        // First order one forward mode.
        // This case is needed if first order forward mode is used.
        // f'(x) = [   0,   0, 2 * x_2 ]
        //         [ x_1, x_0,       0 ]
        // y^1 =  f'(x^0) * x^1
        if (p <= 1) {
            // y_0^1 = 2 * x_2^0 * x_2^1
            ty[0 * q1 + 1] = 2.0 * tx[2 * q1 + 0] * tx[2 * q1 + 1];
            // y_1^1 = x_1^0 * x_0^1 + x_0^0 * x_1^1
            ty[1 * q1 + 1] = tx[1 * q1 + 0] * tx[0 * q1 + 1];
            ty[1 * q1 + 1] += tx[0 * q1 + 0] * tx[1 * q1 + 1];
        }
        if (q <= 1)
            return ok;
        // ------------------------------------------------------------------
        // Second order forward mode.
        // This case is neede if second order forwrd mode is used.
        // f'(x) = [   0,   0, 2 x_2 ]
        //         [ x_1, x_0,     0 ]
        //
        //            [ 0 , 0 , 0 ]                  [ 0 , 1 , 0 ]
        // f_0''(x) = [ 0 , 0 , 0 ]  f_1^{(2)} (x) = [ 1 , 0 , 0 ]
        //            [ 0 , 0 , 2 ]                  [ 0 , 0 , 0 ]
        //
        //  y_0^2 = x^1 * f_0''( x^0 ) x^1 / 2! + f_0'( x^0 ) x^2
        //        = ( x_2^1 * 2.0 * x_2^1 ) / 2!
        //        + 2.0 * x_2^0 * x_2^2
        ty[0 * q1 + 2] = tx[2 * q1 + 1] * tx[2 * q1 + 1];
        ty[0 * q1 + 2] += 2.0 * tx[2 * q1 + 0] * tx[2 * q1 + 2];
        //
        //  y_1^2 = x^1 * f_1''( x^0 ) x^1 / 2! + f_1'( x^0 ) x^2
        //        = ( x_1^1 * x_0^1 + x_0^1 * x_1^1) / 2
        //        + x_1^0 * x_0^2 + x_0^0 + x_1^2
        ty[1 * q1 + 2] = tx[1 * q1 + 1] * tx[0 * q1 + 1];
        ty[1 * q1 + 2] += tx[1 * q1 + 0] * tx[0 * q1 + 2];
        ty[1 * q1 + 2] += tx[0 * q1 + 0] * tx[1 * q1 + 2];
        // ------------------------------------------------------------------
        return ok;
    }

    bool reverse(size_t q,
                 const CppAD::vector<double>& tx,
                 const CppAD::vector<double>& ty,
                 CppAD::vector<double>& px,
                 const CppAD::vector<double>& py) override {
        if (q == 0) {
            px[0] = tx[1] * py[0];
            px[1] = tx[0] * py[0];
            px[2] = (tx[2] + tx[2]) * py[0];
            return true;

        } else if (q == 1) {
            px[1 * q + 1] = py[1 * q + 1] * tx[0 * q + 1];
            px[0 * q + 1] = py[1 * q + 1] * tx[0 * q + 1];
            px[2 * q + 1] = (py[0 * q + 1] + py[0 * q + 1]) * tx[0 * q + 1];
            return true;
        }

        return false;
    }

    bool for_sparse_jac(size_t q,
                        const CppAD::vector<std::set<size_t> >& r,
                        CppAD::vector<std::set<size_t> >& s) override {
        s[0] = std::set<size_t>{2};
        s[1] = std::set<size_t>{0, 1};

        return true;
    }

    bool for_sparse_jac(size_t q,
                        const CppAD::vector<std::set<size_t> >& r,
                        CppAD::vector<std::set<size_t> >& s,
                        const CppAD::vector<double>& x) override {
        return for_sparse_jac(q, r, s);
    }

    bool rev_sparse_jac(size_t q,
                        const CppAD::vector<std::set<size_t> >& rt,
                        CppAD::vector<std::set<size_t> >& st,
                        const CppAD::vector<double>& x) override {

        st[0] = std::set<size_t>{0};
        st[1] = std::set<size_t>{1};
        st[2] = std::set<size_t>{1};

        return true;
    }

    bool rev_sparse_hes(const CppAD::vector<bool>& vx,
                        const CppAD::vector<bool>& s,
                        CppAD::vector<bool>& t,
                        size_t q,
                        const CppAD::vector<std::set<size_t> >& r,
                        const CppAD::vector<std::set<size_t> >& u,
                        CppAD::vector<std::set<size_t> >& v) override {
        return true;
    }

};


int main() {
    /****************************************************************************
     *                               the model
     **************************************************************************/

    // independent variable vector
    CppAD::vector<ADCG> ax(3);
    ax[0] = 1;
    ax[1] = 2;
    ax[2] = 3;
    Independent(ax);


    // dependent variable vector
    CppAD::vector<ADCG> ay(2);

    CustomAtomicExample customAtomicExample;
    CppAD::vector<double> xSparsityTest(3);
    xSparsityTest[0] = 2;
    xSparsityTest[1] = 2;
    xSparsityTest[2] = 2;
    CGAtomicFun<double> cgAtomicFun(customAtomicExample, xSparsityTest, true);

    // the model equation
    cgAtomicFun(ax, ay);
    ADFun<CGD> fun(ax, ay);

    /****************************************************************************
     *                       Create the dynamic library
     *                  (generates and compiles source code)
     **************************************************************************/
    // generates source code
    ModelCSourceGen<double> cgen(fun, "model_cppADCustomAtomicTest");
    cgen.setCreateJacobian(true);
    cgen.setCreateHessian(true);
    cgen.setCreateForwardZero(true);
    cgen.setCreateForwardOne(true);
    cgen.setCreateReverseOne(true);
    cgen.setCreateReverseTwo(true);
    ModelLibraryCSourceGen<double> libcgen(cgen);

    SaveFilesModelLibraryProcessor<double> p2(libcgen);
    p2.saveSourcesTo("tmp");

    // compile source code
    DynamicModelLibraryProcessor<double> p(libcgen);

    GccCompiler<double> compiler;
    std::unique_ptr<DynamicLib<double>> dynamicLib = p.createDynamicLibrary(compiler);
    /***************************************************************************
     *                       Use the dynamic library
     **************************************************************************/

    std::unique_ptr<GenericModel<double>> model = dynamicLib->model("model_cppADCustomAtomicTest");
    model->addAtomicFunction(customAtomicExample);

    std::vector<double> xv{1, 2, 3};

    // print out the result
    std::cout << "CPPAD results: " << std::endl;

    std::cout << "isForwardZeroAvailable: " << model->isForwardZeroAvailable() << std::endl;
    std::vector<double> distance = model->ForwardZero(xv);
    std::cout << "ForwardZero: " << ArrayView<double>(distance) << std::endl;

    std::cout << "isJacobianAvailable: " << model->isJacobianAvailable() << std::endl;
    std::vector<double> jac = model->Jacobian(xv);
    std::cout << "Jacobian: " << std::endl << ArrayView<double>(jac) << std::endl;

    std::cout << "isHessianAvailable: " << model->isHessianAvailable() << std::endl;
    std::vector<double> hess = model->Hessian(xv, 0);
    std::cout << "Hessian 1: " << std::endl << ArrayView<double>(hess) << std::endl;
    hess = model->Hessian(xv, 1);
    std::cout << "Hessian 2: " << std::endl << ArrayView<double>(hess) << std::endl;

}