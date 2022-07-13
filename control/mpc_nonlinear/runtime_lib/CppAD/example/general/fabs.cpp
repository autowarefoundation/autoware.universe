/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-21 Bradley M. Bell

CppAD is distributed under the terms of the
             Eclipse Public License Version 2.0.

This Source Code may also be made available under the following
Secondary License when the conditions for such availability set forth
in the Eclipse Public License, Version 2.0 are satisfied:
      GNU General Public License, Version 2.0 or later.
---------------------------------------------------------------------------- */

/*
$begin fabs.cpp$$
$spell
    fabs
    abs
$$

$section AD Absolute Value Function: Example and Test$$


$srcthisfile%0%// BEGIN C++%// END C++%1%$$

$end
*/
// BEGIN C++

# include <cppad/cppad.hpp>

bool fabs(void)
{   bool ok = true;

    using CppAD::AD;
    using CppAD::NearEqual;

    // domain space vector
    size_t n = 1;
    CPPAD_TESTVECTOR(AD<double>) ax(n);
    ax[0] = 0.;

    // declare independent variables and start tape recording
    CppAD::Independent(ax);

    // range space vector
    size_t m = 3;
    CPPAD_TESTVECTOR(AD<double>) ay(m);
    ay[0]     = fabs(ax[0] - 1.);
    ay[1]     = fabs(ax[0]);
    ay[2]     = fabs(ax[0] + 1.);
    //
    // create f: x -> y and stop tape recording
    CppAD::ADFun<double> f(ax, ay);

    // check values
    ok &= (ay[0] == 1.);
    ok &= (ay[1] == 0.);
    ok &= (ay[2] == 1.);
    //
    // forward computation of partials w.r.t. a positive x[0] direction
    size_t p = 1;
    CPPAD_TESTVECTOR(double) dx(n), dy(m);
    dx[0] = 1.;
    dy    = f.Forward(p, dx);
    ok  &= (dy[0] == - dx[0]);
    ok  &= (dy[1] ==   0.   );
    ok  &= (dy[2] == + dx[0]);
    //
    // forward computation of partials w.r.t. a negative x[0] direction
    dx[0] = -1.;
    dy    = f.Forward(p, dx);
    ok  &= (dy[0] == - dx[0]);
    ok  &= (dy[1] ==   0.   );
    ok  &= (dy[2] == + dx[0]);
    //
    // reverse computation of derivative of y[0]
    p    = 1;
    CPPAD_TESTVECTOR(double)  w(m), dw(n);
    w[0] = 1.; w[1] = 0.; w[2] = 0;
    dw   = f.Reverse(p, w);
    ok  &= (dw[0] == -1.);

    // reverse computation of derivative of y[1]
    w[0] = 0.; w[1] = 1.; w[2] = 0;
    dw   = f.Reverse(p, w);
    ok  &= (dw[0] == 0.);

    // reverse computation of derivative of y[3]
    w[0] = 0.; w[1] = 0.; w[2] = 1;
    dw   = f.Reverse(p, w);
    ok  &= (dw[0] == 1.);

    // use a VecAD<Base>::reference object with fabs
    CppAD::VecAD<double> av(1);
    AD<double> az(0);
    av[az]   = -1;
    AD<double> a_result = fabs(av[az]);
    ok  &= a_result == 1.0;

    return ok;
}

// END C++
