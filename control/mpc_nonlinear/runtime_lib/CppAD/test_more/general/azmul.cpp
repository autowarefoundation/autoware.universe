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
# include <cmath>

namespace {
    bool test_base2ad(void)
    {   bool ok = true;
        double eps99 = 99.0 * std::numeric_limits<double>::epsilon();

        using CppAD::AD;

        // Both recordiings are done with the dynamic  parameter p = 0, 1
        // to make sure does not short circut multiply
        for(size_t ip = 0; ip < 2; ++ip)
        {
            // f(p; x) = p[0] * x[0] * x[0]
            CPPAD_TESTVECTOR(AD<double>)  ap(1), ax(1), ay(1), aw(1);
            ap[0] = double(ip);
            ax[0] = 0.0;
            CppAD::Independent(ax, ap);
            ay[0] = ap[0] * ax[0] * ax[0];
            CppAD::ADFun<double> f(ax, ay);

            // AD version of f
            CppAD::ADFun< AD<double> , double > af = f.base2ad();

            // g(p; x) = d/dx f(p, x) = 2 * p[0] * x[0]
            CppAD::Independent(ax, ap);
            af.new_dynamic(ap);
            af.Forward(0, ax);
            aw[0] = 1.0;
            ay    = af.Reverse(1, aw);
            CppAD::ADFun<double> g(ax, ay);

            // Evaluate g(p, x)
            CPPAD_TESTVECTOR(double) p(1), x(1), y(1);
            p[0] = 2.0;
            x[0] = 3.0;
            g.new_dynamic(p);
            y = g.Forward(0, x);

            // check result
            double check = 2.0 * p[0] * x[0];
            ok &= CppAD::NearEqual(y[0], check, eps99, eps99);
        }
        return ok;
    }
    bool test_forward(void)
    {   bool ok = true;

        using CppAD::AD;
        using CppAD::NearEqual;
        double inf = std::numeric_limits<double>::infinity();
        double eps = 10. * std::numeric_limits<double>::epsilon();

        typedef AD<double>    a1double;
        typedef AD<a1double>  a2double;

        // domain space vector
        size_t n = 2;
        size_t m = 3;

        // double level
        CPPAD_TESTVECTOR(double) x(n), z(m);
        x[0] = 3.0;
        x[1] = 4.0;

        // start a1 level recording
        CPPAD_TESTVECTOR(a1double) a1x(n), a1dx(n), a1y(m), a1z(m);
        for(size_t j = 0; j < n; j++)
            a1x[j] = x[j];
        CppAD::Independent(a1x);

        // start a2 level recording
        CPPAD_TESTVECTOR(a2double) a2x(n), a2y(m);
        for(size_t j = 0; j < n; j++)
            a2x[j] = a1x[j];
        CppAD::Independent(a2x);

        // y
        a2y[0] = CppAD::azmul(a2x[0], a2x[1]); // azmul(variable, variable)
        a2y[1] = CppAD::azmul(a1x[0], a2x[1]); // azmul(parameter, variable)
        a2y[2] = CppAD::azmul(a2x[0], a1x[1]); // azmul(variable, parameter)

        // create f: x -> y and stop a2 recording
        CppAD::ADFun<a1double> a1f;
        a1f.Dependent(a2x, a2y);

        // check y
        a1y = a1f.Forward(0, a1x);  // azmul(variable, variable) only
        for(size_t i = 0; i < m; i++)
            ok &= NearEqual(a1y[i] , x[0] * x[1],  eps, eps);

        for(size_t j = 0; j < n; j++)
            a1dx[j] = a1double(1.0);
        a1z = a1f.Forward(1, a1dx);

        // create g: x -> z and stop a1 recording
        CppAD::ADFun<double> g;
        g.Dependent(a1x, a1z);

        // check value when x[0] is not zero
        z = g.Forward(0, x);

        // z_0 = d_lambda [ ( x[0] + lambda ) * ( x[1] + lambda ) ]
        ok &= NearEqual(z[0] , x[0] + x[1],  eps, eps);

        // z_1 = d_lambda [ x[0] * ( x[1] + lambda ) ]
        ok &= NearEqual(z[1] , x[0],  eps, eps);

        // z_2 = d_lambda [ ( x[0] + lambda ) * x[1] ]
        ok &= NearEqual(z[2] , x[1],  eps, eps);

        // check value x[0] is zero and x[1] is infinity
        x[0] = 0.0;
        x[1] = inf;
        z    = g.Forward(0, x);
        ok  &= z[0] == inf;
        ok  &= z[1] == 0.0;
        ok  &= z[2] == inf;

        return ok;
    }
    bool test_reverse(void)
    {   bool ok = true;

        using CppAD::AD;
        using CppAD::NearEqual;
        double inf = std::numeric_limits<double>::infinity();
        double eps = 10. * std::numeric_limits<double>::epsilon();

        typedef AD<double>    a1double;
        typedef AD<a1double>  a2double;

        // domain space vector
        size_t n = 2;
        size_t m = 3;

        // double level
        CPPAD_TESTVECTOR(double) x(n), z(n);
        x[0] = 3.0;
        x[1] = 4.0;

        // start a1 level recording
        CPPAD_TESTVECTOR(a1double) a1x(n), a1dx(n), a1w(m), a1z(n);
        for(size_t j = 0; j < n; j++)
            a1x[j] = x[j];
        CppAD::Independent(a1x);

        // start a2 level recording
        CPPAD_TESTVECTOR(a2double) a2x(n), a2y(m);
        for(size_t j = 0; j < n; j++)
            a2x[j] = a1x[j];
        CppAD::Independent(a2x);

        // y
        a2y[0] = CppAD::azmul(a2x[0], a2x[1]); // azmul(variable, variable)
        a2y[1] = CppAD::azmul(a1x[0], a2x[1]); // azmul(parameter, variable)
        a2y[2] = CppAD::azmul(a2x[0], a1x[1]); // azmul(variable, parameter)

        // create f: x -> y and stop a2 recording
        CppAD::ADFun<a1double> a1f;
        a1f.Dependent(a2x, a2y);

        // w(x) = y[0] + y[1] + y[2]
        for(size_t i = 0; i < m; i++)
            a1w[i] = a1double(1.0);
        a1f.Forward(0, a1x);
        a1dx = a1f.Reverse(1, a1w);

        // create g: x -> z and stop a1 recording
        CppAD::ADFun<double> g;
        g.Dependent(a1x, a1dx);

        // check value when x[0] is not zero
        z = g.Forward(0, x);

        // partial y[0] w.r.t x[0] = x[1]
        // partial y[1] w.r.t x[0] = 0
        // partial y[2] w.r.t x[0] = x[1]
        ok &= NearEqual(z[0] , x[1] + x[1],  eps, eps);

        // partial y[0] w.r.t x[1] = x[0]
        // partial y[1] w.r.t x[1] = x[0]
        // partial y[2] w.r.t x[1] = 0
        ok &= NearEqual(z[1] , x[0] + x[0],  eps, eps);

        // check value x[0] is zero and x[1] is infinity
        x[0] = 0.0;
        x[1] = inf;
        z    = g.Forward(0, x);
        ok  &= z[0] == inf;
        ok  &= z[1] == 0.0;

        return ok;
    }
    bool test_forward_dir(void)
    {   bool ok = true;

        using CppAD::AD;
        using CppAD::NearEqual;
        double inf = std::numeric_limits<double>::infinity();
        double eps = 10. * std::numeric_limits<double>::epsilon();

        typedef AD<double>    a1double;
        typedef AD<a1double>  a2double;

        // domain space vector
        size_t n = 2;
        size_t m = 3;
        size_t r = 2;

        // double level
        CPPAD_TESTVECTOR(double) x(n), z(r * m);
        x[0] = 3.0;
        x[1] = 4.0;

        // start a1 level recording
        CPPAD_TESTVECTOR(a1double) a1x(n), a1dx(r * n), a1y(m), a1z(r * m);
        for(size_t j = 0; j < n; j++)
            a1x[j] = x[j];
        CppAD::Independent(a1x);

        // start a2 level recording
        CPPAD_TESTVECTOR(a2double) a2x(n), a2y(m);
        for(size_t j = 0; j < n; j++)
            a2x[j] = a1x[j];
        CppAD::Independent(a2x);

        // y
        a2y[0] = CppAD::azmul(a2x[0], a2x[1]); // azmul(variable, variable)
        a2y[1] = CppAD::azmul(a1x[0], a2x[1]); // azmul(parameter, variable)
        a2y[2] = CppAD::azmul(a2x[0], a1x[1]); // azmul(variable, parameter)

        // create f: x -> y and stop a2 recording
        CppAD::ADFun<a1double> a1f;
        a1f.Dependent(a2x, a2y);

        // check y
        a1y = a1f.Forward(0, a1x);  // azmul(variable, variable) only
        for(size_t i = 0; i < m; i++)
            ok &= NearEqual(a1y[i] , x[0] * x[1],  eps, eps);

        for(size_t j = 0; j < n; j++)
        {   for(size_t ell = 0; ell < r; ell++)
                a1dx[r * j + ell] = a1double(1.0 + double(ell));
        }
        a1z = a1f.Forward(1, r, a1dx);


        // create g: x -> z and stop a1 recording
        CppAD::ADFun<double> g;
        g.Dependent(a1x, a1z);

        // check value when x[0] is not zero
        z = g.Forward(0, x);

        // z_00 = d_lambda [ ( x[0] + lambda ) * ( x[1] + lambda ) ]
        ok &= NearEqual(z[r * 0 + 0] , x[0] + x[1],  eps, eps);

        // z_01 = d_lambda [ ( x[0] + 2 * lambda ) * ( x[1] + 2 * lambda ) ]
        ok &= NearEqual(z[r * 0 + 1] , 2.0*(x[0] + x[1]),  eps, eps);

        // z_10 = d_lambda [ x[0] * ( x[1] + lambda ) ]
        ok &= NearEqual(z[r * 1 + 0] , x[0],  eps, eps);

        // z_11 = d_lambda [ x[0] * ( x[1] + 2 * lambda ) ]
        ok &= NearEqual(z[r * 1 + 1] , 2.0 * x[0],  eps, eps);

        // z_20 = d_lambda [ ( x[0] + lambda ) * x[1] ]
        ok &= NearEqual(z[r * 2 + 0] , x[1],  eps, eps);

        // z_21 = d_lambda [ ( x[0] + 2 * lambda ) * x[1] ]
        ok &= NearEqual(z[r * 2 + 1] , 2.0 * x[1],  eps, eps);

        // check value x[0] is zero and x[1] is infinity
        x[0] = 0.0;
        x[1] = inf;
        z    = g.Forward(0, x);
        ok  &= z[r * 0 + 0] == inf;
        ok  &= z[r * 0 + 1] == inf;
        ok  &= z[r * 1 + 0] == 0.0;
        ok  &= z[r * 1 + 1] == 0.0;
        ok  &= z[r * 2 + 0] == inf;
        ok  &= z[r * 2 + 1] == inf;

        return ok;
    }
}
bool azmul(void)
{   bool ok = true;

    ok &= test_base2ad();
    ok &= test_forward();
    ok &= test_reverse();
    ok &= test_forward_dir();

    return ok;
}
