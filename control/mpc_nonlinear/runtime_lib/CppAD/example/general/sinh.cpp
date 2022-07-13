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
$begin sinh.cpp$$
$spell
    sinh
$$

$section The AD sinh Function: Example and Test$$


$srcthisfile%0%// BEGIN C++%// END C++%1%$$

$end
*/
// BEGIN C++

# include <cppad/cppad.hpp>
# include <cmath>

bool Sinh(void)
{   bool ok = true;

    using CppAD::AD;
    using CppAD::NearEqual;
    double eps99 = 99.0 * std::numeric_limits<double>::epsilon();

    // domain space vector
    size_t n  = 1;
    double x0 = 0.5;
    CPPAD_TESTVECTOR(AD<double>) x(n);
    x[0]      = x0;

    // declare independent variables and start tape recording
    CppAD::Independent(x);

    // range space vector
    size_t m = 1;
    CPPAD_TESTVECTOR(AD<double>) y(m);
    y[0] = CppAD::sinh(x[0]);

    // create f: x -> y and stop tape recording
    CppAD::ADFun<double> f(x, y);

    // check value
    double check = std::sinh(x0);
    ok &= NearEqual(y[0] , check, eps99, eps99);

    // forward computation of first partial w.r.t. x[0]
    CPPAD_TESTVECTOR(double) dx(n);
    CPPAD_TESTVECTOR(double) dy(m);
    dx[0] = 1.;
    dy    = f.Forward(1, dx);
    check = std::cosh(x0);
    ok   &= NearEqual(dy[0], check, eps99, eps99);

    // reverse computation of derivative of y[0]
    CPPAD_TESTVECTOR(double)  w(m);
    CPPAD_TESTVECTOR(double) dw(n);
    w[0]  = 1.;
    dw    = f.Reverse(1, w);
    ok   &= NearEqual(dw[0], check, eps99, eps99);

    // use a VecAD<Base>::reference object with sinh
    CppAD::VecAD<double> v(1);
    AD<double> zero(0);
    v[zero]           = x0;
    AD<double> result = CppAD::sinh(v[zero]);
    check = std::sinh(x0);
    ok   &= NearEqual(result, check, eps99, eps99);

    return ok;
}

// END C++
