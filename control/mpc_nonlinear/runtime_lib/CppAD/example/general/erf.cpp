/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-20 Bradley M. Bell

CppAD is distributed under the terms of the
             Eclipse Public License Version 2.0.

This Source Code may also be made available under the following
Secondary License when the conditions for such availability set forth
in the Eclipse Public License, Version 2.0 are satisfied:
      GNU General Public License, Version 2.0 or later.
---------------------------------------------------------------------------- */

/*
$begin erf.cpp$$
$spell
    tan
    erf
$$

$section The AD erf Function: Example and Test$$


$srcthisfile%0%// BEGIN C++%// END C++%1%$$

$end
*/
// BEGIN C++

# include <cppad/cppad.hpp>
# include <cmath>
# include <limits>

bool erf(void)
{   bool ok = true;

    using CppAD::AD;
    using CppAD::NearEqual;
    double eps = 10. * CppAD::numeric_limits<double>::epsilon();

    // domain space vector
    size_t n  = 1;
    double x0 = 0.5;
    CPPAD_TESTVECTOR(AD<double>) ax(n);
    ax[0]     = x0;

    // declare independent variables and start tape recording
    CppAD::Independent(ax);

    // range space vector
    size_t m = 1;
    CPPAD_TESTVECTOR(AD<double>) ay(m);
    ay[0] = CppAD::erf(ax[0]);

    // create f: x -> y and stop tape recording
    CppAD::ADFun<double> f(ax, ay);

    // check relative erorr
    double erf_x0 = 0.5204998778130465;
    ok &= NearEqual(ay[0] , erf_x0,  0.,    4e-4);
    double tmp = std::max(1e-15, eps);
    ok &= NearEqual(ay[0] , erf_x0,  0.,    tmp);

    // value of derivative of erf at x0
    double pi     = 4. * std::atan(1.);
    double factor = 2. / sqrt(pi);
    double check  = factor * std::exp(-x0 * x0);

    // forward computation of first partial w.r.t. x[0]
    CPPAD_TESTVECTOR(double) dx(n);
    CPPAD_TESTVECTOR(double) dy(m);
    dx[0] = 1.;
    dy    = f.Forward(1, dx);
    ok   &= NearEqual(dy[0], check,  0.,  1e-3);
    ok   &= NearEqual(dy[0], check,  0.,  eps);

    // reverse computation of derivative of y[0]
    CPPAD_TESTVECTOR(double)  w(m);
    CPPAD_TESTVECTOR(double) dw(n);
    w[0]  = 1.;
    dw    = f.Reverse(1, w);
    ok   &= NearEqual(dw[0], check,  0., 1e-1);
    ok   &= NearEqual(dw[0], check,  0., eps);

    // use a VecAD<Base>::reference object with erf
    CppAD::VecAD<double> v(1);
    AD<double> zero(0);
    v[zero]           = x0;
    AD<double> result = CppAD::erf(v[zero]);
    ok   &= NearEqual(result, ay[0], eps, eps);

    // use a double with erf
    ok   &= NearEqual(CppAD::erf(x0), ay[0], eps, eps);

    return ok;
}

// END C++
