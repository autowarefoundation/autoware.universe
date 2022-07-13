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
$begin unary_minus.cpp$$
$spell
    Cpp
    cstddef
$$

$section AD Unary Minus Operator: Example and Test$$


$srcthisfile%0%// BEGIN C++%// END C++%1%$$

$end
*/
// BEGIN C++

# include <cppad/cppad.hpp>

bool UnaryMinus(void)
{   bool ok = true;
    using CppAD::AD;


    // domain space vector
    size_t n = 1;
    CPPAD_TESTVECTOR(AD<double>) x(n);
    x[0]      = 3.;

    // declare independent variables and start tape recording
    CppAD::Independent(x);

    // range space vector
    size_t m = 1;
    CPPAD_TESTVECTOR(AD<double>) y(m);
    y[0] = - x[0];

    // create f: x -> y and stop tape recording
    CppAD::ADFun<double> f(x, y);

    // check values
    ok &= ( y[0] == -3. );

    // forward computation of partials w.r.t. x[0]
    CPPAD_TESTVECTOR(double) dx(n);
    CPPAD_TESTVECTOR(double) dy(m);
    size_t p = 1;
    dx[0]    = 1.;
    dy       = f.Forward(p, dx);
    ok      &= ( dy[0] == -1. );   // dy[0] / dx[0]

    // reverse computation of dertivative of y[0]
    CPPAD_TESTVECTOR(double)  w(m);
    CPPAD_TESTVECTOR(double) dw(n);
    w[0] = 1.;
    dw   = f.Reverse(p, w);
    ok &= ( dw[0] == -1. );       // dy[0] / dx[0]

    // use a VecAD<Base>::reference object with unary minus
    CppAD::VecAD<double> v(1);
    AD<double> zero(0);
    v[zero] = x[0];
    AD<double> result = - v[zero];
    ok     &= (result == y[0]);

    return ok;
}
// END C++
