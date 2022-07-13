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
$begin forward_order.cpp$$
$spell
    Cpp
$$

$section Forward Mode: Example and Test of Multiple Orders$$

$srcthisfile%0%// BEGIN C++%// END C++%1%$$

$end
*/
// BEGIN C++
# include <limits>
# include <cppad/cppad.hpp>
bool forward_order(void)
{   bool ok = true;
    using CppAD::AD;
    using CppAD::NearEqual;
    double eps = 10. * std::numeric_limits<double>::epsilon();

    // domain space vector
    size_t n = 2;
    CPPAD_TESTVECTOR(AD<double>) ax(n);
    ax[0] = 0.;
    ax[1] = 1.;

    // declare independent variables and starting recording
    CppAD::Independent(ax);

    // range space vector
    size_t m = 1;
    CPPAD_TESTVECTOR(AD<double>) ay(m);
    ay[0] = ax[0] * ax[0] * ax[1];

    // create f: x -> y and stop tape recording
    CppAD::ADFun<double> f(ax, ay);

    // initially, the variable values during taping are stored in f
    ok &= f.size_order() == 1;

    // Compute three forward orders at once
    size_t q = 2, q1 = q+1;
    CPPAD_TESTVECTOR(double) xq(n*q1), yq;
    xq[q1*0 + 0] = 3.;    xq[q1*1 + 0] = 4.; // x^0 (order zero)
    xq[q1*0 + 1] = 1.;    xq[q1*1 + 1] = 0.; // x^1 (order one)
    xq[q1*0 + 2] = 0.;    xq[q1*1 + 2] = 0.; // x^2 (order two)
    // X(t) =   x^0 + x^1 * t + x^2 * t^2
    //      = [ 3 + t, 4 ]
    yq  = f.Forward(q, xq);
    ok &= size_t( yq.size() ) == m*q1;
    // Y(t) = F[X(t)]
    //      = (3 + t) * (3 + t) * 4
    //      = y^0 + y^1 * t + y^2 * t^2 + o(t^3)
    //
    // check y^0 (order zero)
    CPPAD_TESTVECTOR(double) x0(n);
    x0[0] = xq[q1*0 + 0];
    x0[1] = xq[q1*1 + 0];
    ok  &= NearEqual(yq[q1*0 + 0] , x0[0]*x0[0]*x0[1], eps, eps);
    //
    // check y^1 (order one)
    ok  &= NearEqual(yq[q1*0 + 1] , 2.*x0[0]*x0[1], eps, eps);
    //
    // check y^2 (order two)
    double F_00 = 2. * yq[q1*0 + 2]; // second partial F w.r.t. x_0, x_0
    ok   &= NearEqual(F_00, 2.*x0[1], eps, eps);
    //
    // check number of orders per variable
    ok   &= f.size_order() == 3;
    //
    return ok;
}
// END C++
