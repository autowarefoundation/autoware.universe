/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-20 Bradley M. Bell

CppAD is distributed under the terms of the
             Eclipse Public License Version 2.0.

This Source Code may also be made available under the following
Secondary License when the conditions for such availability set forth
in the Eclipse Public License, Version 2.0 are satisfied:
      GNU General Public License, Version 2.0 or later.
---------------------------------------------------------------------------- */

# include <cppad/cppad.hpp>

namespace {
    // ---------------------------------------------------------------------
    bool old_example(void)
    {   bool ok = true;
        using namespace CppAD;
        using CppAD::atan;
        using CppAD::exp;
        using CppAD::sqrt;
        double eps = 100.0 * std::numeric_limits<double>::epsilon();
        // Construct function object corresponding to erf
        CPPAD_TESTVECTOR(AD<double>) ax(1);
        CPPAD_TESTVECTOR(AD<double>) ay(1);
        ax[0] = 0.;
        Independent(ax);
        ay[0] = erf(ax[0]);
        ADFun<double> f(ax, ay);

        // Construct function object corresponding to derivative of erf
        Independent(ax);
        double pi = 4.0 * atan(1.0);
        ay[0] = exp( - ax[0] * ax[0] ) * 2.0 / sqrt(pi);
        ADFun<double> df(ax, ay);

        // vectors to use with function object
        CPPAD_TESTVECTOR(double) x0(1), y0(1), x1(1), y1(1), check(1);

        // check value at zero
        x0[0]    = 1.5;
        y0       = f.Forward(0, x0);
        check[0] = 0.96611;
        ok      &= std::fabs(check[0] - y0[0]) <= 4e-4;

        // check the derivative of error function
        x1[0] = 1.0;
        y1    = f.Forward(1, x1);
        check = df.Forward(0, x0);
        ok   &= NearEqual(check[0], y1[0], 0., 2e-3);
        ok   &= NearEqual(check[0], y1[0], eps, eps);

        // check second derivative
        CPPAD_TESTVECTOR(double) x2(1), y2(1);
        x2[0] = 0.0;
        y2    = f.Forward(2, x2);
        check = df.Forward(1, x1);
        ok   &= NearEqual(check[0] / 2.0, y2[0], 0., 2e-3);
        ok   &= NearEqual(check[0] / 2.0, y2[0], eps, eps);

        // check third derivative
        CPPAD_TESTVECTOR(double) x3(1), y3(1);
        x3[0] = 0.0;
        y3    = f.Forward(3, x3);
        check = df.Forward(2, x2);
        ok   &= NearEqual(check[0] / 3.0, y3[0], 0., 2e-3);
        ok   &= NearEqual(check[0] / 3.0, y3[0], eps, eps);

        // check 4-th order of reverse mode
        CPPAD_TESTVECTOR(double) w(1), dy(4), x4(1), y4(1);
        x4[0] = 0.0;
        w[0]  = 1.0;
        dy    = f.Reverse(4, w);
        y4    = f.Forward(4, x4);
        //
        ok  &= NearEqual(dy[0], y1[0], 0., 2e-3);
        ok  &= NearEqual(dy[0], y1[0], eps, eps);
        //
        ok  &= NearEqual(dy[1], 2.0 * y2[0], 0., 2e-3);
        ok  &= NearEqual(dy[1], 2.0 * y2[0], eps, eps);
        //
        ok  &= NearEqual(dy[2], 3.0 * y3[0], 0., 2e-3);
        ok  &= NearEqual(dy[2], 3.0 * y3[0], eps, eps);
        //
        ok  &= NearEqual(dy[3], 4.0 * y4[0], 0., 2e-3);
        ok  &= NearEqual(dy[3], 4.0 * y4[0], eps, eps);

        return ok;
    }
    // ---------------------------------------------------------------------
    bool hessian(void)
    {   bool ok = true;
        double eps = 1.0 * std::numeric_limits<double>::epsilon();
        using CppAD::vector;
        using CppAD::AD;

        size_t n = 2;
        size_t m = 1;
        vector<double> x(n), w(m);
        w[0] = 1.0;
        vector< AD<double> > ax(n), ay(m);
        ax[0] = x[0] = 0.5;
        ax[1] = x[1] = 0.0;

        // construct function
        CppAD::Independent(ax);
        ay[0] = erf( 2.0 * ax[0] );
        CppAD::ADFun<double> f(ax, ay);

        // dense hessian
        vector<double> dense_hess = f.Hessian(x, 0);

        // sparse_hessian
        vector<double> sparse_hess = f.SparseHessian(x, w);

        // Define g(u) = erf(2 * u)
        // g'(u)   = 2 * erf'(2 * u)
        //         = 2 * exp( - 2 * u * 2 * u ) * 2 / sqrt(pi)
        //         = exp( - 4 * u * u ) * 4 / sqrt(pi)
        // g''(u)  = - exp( - 4 * u * u ) * 32 * u / sqrt(pi)
        double root_pi = std::sqrt( 4.0 * atan(1.0) );
        double check   = -std::exp(-4.0 * x[0] * x[0]) * 32.0 * x[0] / root_pi;

        ok &= CppAD::NearEqual(dense_hess[0], check, eps, eps);
        ok &= CppAD::NearEqual(sparse_hess[0], check, eps, eps);

        for(size_t k = 1; k < n * n; k++)
        {   ok &= CppAD::NearEqual(dense_hess[k], 0.0, eps, eps);
            ok &= CppAD::NearEqual(sparse_hess[k], 0.0, eps, eps);
        }
        return ok;
    }
    // ---------------------------------------------------------------------
    bool mul_dir(void)
    {   bool ok = true;
        using namespace CppAD;
        using CppAD::atan;
        using CppAD::exp;
        using CppAD::sqrt;
        double eps = 100.0 * std::numeric_limits<double>::epsilon();

        // Construct function object corresponding to erf
        CPPAD_TESTVECTOR(AD<double>) ax(1);
        CPPAD_TESTVECTOR(AD<double>) ay(1);
        ax[0] = 0.;
        Independent(ax);
        ay[0] = erf(ax[0]);
        ADFun<double> f(ax, ay);

        // Construct function object corresponding to derivative of erf
        Independent(ax);
        double pi = 4.0 * atan(1.0);
        ay[0] = exp( - ax[0] * ax[0] ) * 2.0 / sqrt(pi);
        ADFun<double> df(ax, ay);

        // number of directions
        size_t r = 1;

        // vectors to use with objects
        CPPAD_TESTVECTOR(double) x0(1), y0(1), x1(1), y1(1), y2(1), y3(1);
        CPPAD_TESTVECTOR(double) zero(1), check(1);
        CPPAD_TESTVECTOR(double) xq(r), yq(r), checkq(r), zeroq(r);

        // check function value
        x0[0]      = 1.5;
        y0         = f.Forward(0, x0);
        check[0]   = 0.9661051464753108;
        double tmp = std::max(1e-15, eps);
        ok        &= NearEqual(check[0], y0[0], 0.0, tmp);

        // check first order derivative
        x1[0] = 1.0;
        y1    = f.Forward(1, x1);
        check = df.Forward(0, x0);
        ok   &= NearEqual(check[0], y1[0], eps, eps);
        for(size_t ell = 0; ell < r; ell++)
        {   xq[ell]     = x1[ell] / double(ell + 1);
            zeroq[ell]  = 0.0;
        }
        yq    = f.Forward(1, r, xq);
        for(size_t ell = 0; ell < r; ell++)
        {   checkq[ell] = check[0] * xq[ell];
            ok         &= NearEqual(checkq[ell], yq[ell], eps, eps);
        }

        // check second order derivative
        zero[0]   = 0.0;
        y2        = f.Forward(2, zero);
        check     = df.Forward(1, x1);
        check[0] /= 2.0;
        ok       &= NearEqual(check[0], y2[0], eps, eps);
        yq        = f.Forward(2, r, zeroq);
        for(size_t ell = 0; ell < r; ell++)
        {   checkq[ell] = check[0] * xq[ell];
            ok         &= NearEqual(checkq[ell], yq[ell], eps, eps);
        }

        // check third order derivative
        zero[0]   = 0.0;
        y3        = f.Forward(3, zero);
        check     = df.Forward(2, zero);
        check[0] /= 3.0;
        ok       &= NearEqual(check[0], y3[0], eps, eps);
        yq        = f.Forward(3, r, zeroq);
        for(size_t ell = 0; ell < r; ell++)
        {   checkq[ell] = check[0] * xq[ell];
            ok         &= NearEqual(checkq[ell], yq[ell], eps, eps);
        }

        return ok;
    }
    // -------------------------------------------------------------------
}
bool erf(void)
{   bool ok = true;
    ok     &= old_example();
    ok     &= hessian();
    ok     &= mul_dir();
    return ok;
}
