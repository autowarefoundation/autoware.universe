/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-17 Bradley M. Bell

CppAD is distributed under the terms of the
             Eclipse Public License Version 2.0.

This Source Code may also be made available under the following
Secondary License when the conditions for such availability set forth
in the Eclipse Public License, Version 2.0 are satisfied:
      GNU General Public License, Version 2.0 or later.
---------------------------------------------------------------------------- */


/*
Test higher order derivatives for tan(x) function.
*/

# include <cppad/cppad.hpp>

namespace {
    bool tan_two(void)
    {   bool ok = true;
        using CppAD::AD;
        using CppAD::NearEqual;
        double eps = 10. * std::numeric_limits<double>::epsilon();

        // domain space vector
        size_t n = 1;
        CPPAD_TESTVECTOR(AD<double>) ax(n);
        ax[0] = 0.5;

        // declare independent variables and starting recording
        CppAD::Independent(ax);

        // range space vector
        size_t m = 1;
        CPPAD_TESTVECTOR(AD<double>) ay(m);
        ay[0] = tan( ax[0] );

        // create f: x -> y and stop tape recording
        CppAD::ADFun<double> f(ax, ay);

        // first order Taylor coefficient
        CPPAD_TESTVECTOR(double) x1(n), y1;
        x1[0] = 2.0;
        y1    = f.Forward(1, x1);
        ok   &= size_t( y1.size() ) == m;

        // secondorder Taylor coefficients
        CPPAD_TESTVECTOR(double) x2(n), y2;
        x2[0] = 0.0;
        y2    = f.Forward(2, x2);
        ok   &= size_t( y2.size() ) == m;
        //
        // Y  (t)    = F[X_0(t)]
        //           =  tan(0.5 + 2t )
        // Y' (t)    =  2 * cos(0.5 + 2t )^(-2)
        double sec_sq  = 1.0 / ( cos(0.5) * cos(0.5) );
        double check   = 2.0 * sec_sq;
        ok  &= NearEqual(y1[0] , check, eps, eps);
        //
        // Y''(0)    = 8*cos(0.5)^(-3)*sin(0.5)
        check = 8.0 * tan(0.5) * sec_sq / 2.0;
        ok    &= NearEqual(y2[0] , check, eps, eps);
        //
        return ok;
    }
    bool tan_case(bool tan_first)
    {   bool ok = true;
        double eps = 100. * std::numeric_limits<double>::epsilon();
        using CppAD::AD;
        using CppAD::NearEqual;

        // independent variable vector, indices, values, and declaration
        size_t n = 1;
        CPPAD_TESTVECTOR(AD<double>) ax(n);
        ax[0]     = .7;
        Independent(ax);

        // dependent variable vector and indices
        size_t m = 1;
        CPPAD_TESTVECTOR(AD<double>) ay(m);
        if( tan_first )
            ay[0] = atan( tan( ax[0] ) );
        else
            ay[0] = tan( atan( ax[0] ) );

        // check value
        ok &= NearEqual(ax[0] , ay[0],  eps, eps);

        // create f: x -> y and vectors used for derivative calculations
        CppAD::ADFun<double> f(ax, ay);
        CPPAD_TESTVECTOR(double) dx(n), dy(m);

        // forward computation of partials w.r.t. x
        dx[0] = 1.;
        dy    = f.Forward(1, dx);
        ok   &= NearEqual(dy[0], 1e0, eps, eps);
        size_t p, order = 5;
        dx[0] = 0.;
        for(p = 2; p < order; p++)
        {   dy    = f.Forward(p, dx);
            ok   &= NearEqual(dy[0], 0e0, eps, eps);
        }

        // reverse computation of order partial
        CPPAD_TESTVECTOR(double)  w(m), dw(n * order);
        w[0] = 1.;
        dw   = f.Reverse(order, w);
        ok   &= NearEqual(dw[0], 1e0, eps, eps);
        for(p = 1; p < order; p++)
            ok   &= NearEqual(dw[p], 0e0, eps, eps);

        return ok;
    }
    bool tanh_case(bool tanh_first)
    {   bool ok = true;
        double eps = 100. * std::numeric_limits<double>::epsilon();
        using CppAD::AD;
        using CppAD::NearEqual;

        // independent variable vector, indices, values, and declaration
        size_t n = 1;
        CPPAD_TESTVECTOR(AD<double>) ax(n);
        ax[0]     = .5;
        Independent(ax);

        // dependent variable vector and indices
        size_t m = 1;
        CPPAD_TESTVECTOR(AD<double>) ay(m);
        AD<double> z;
        if( tanh_first )
        {   z     = tanh( ax[0] );
            ay[0] = .5 * log( (1. + z) / (1. - z) );
        }
        else
        {   z     = .5 * log( (1. + ax[0]) / (1. - ax[0]) );
            ay[0] = tanh(z);
        }
        // check value
        ok &= NearEqual(ax[0] , ay[0],  eps, eps);

        // create f: x -> y and vectors used for derivative calculations
        CppAD::ADFun<double> f(ax, ay);
        CPPAD_TESTVECTOR(double) dx(n), dy(m);

        // forward computation of partials w.r.t. x
        dx[0] = 1.;
        dy    = f.Forward(1, dx);
        ok   &= NearEqual(dy[0], 1e0, eps, eps);
        size_t p, order = 5;
        dx[0] = 0.;
        for(p = 2; p < order; p++)
        {   dy    = f.Forward(p, dx);
            ok   &= NearEqual(dy[0], 0e0, eps, eps);
        }

        // reverse computation of order partial
        CPPAD_TESTVECTOR(double)  w(m), dw(n * order);
        w[0] = 1.;
        dw   = f.Reverse(order, w);
        ok   &= NearEqual(dw[0], 1e0, eps, eps);
        for(p = 1; p < order; p++)
            ok   &= NearEqual(dw[p], 0e0, eps, eps);

        return ok;
    }
}
bool tan(void)
{   bool ok = true;
    //
    ok     &= tan_case(true);
    ok     &= tan_case(false);
    ok     &= tanh_case(true);
    ok     &= tanh_case(false);
    //
    ok     &= tan_two();
    return ok;
}
