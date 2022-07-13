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
$begin forward.cpp$$
$spell
    Cpp
$$

$section Forward Mode: Example and Test$$

$srcthisfile%0%// BEGIN C++%// END C++%1%$$

$end
*/
// BEGIN C++
# include <limits>
# include <cppad/cppad.hpp>
namespace { // --------------------------------------------------------
// define the template function ForwardCases<Vector> in empty namespace
template <class Vector>
bool ForwardCases(void)
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

    // zero order forward mode using notation in forward_zero
    // use the template parameter Vector for the vector type
    Vector x0(n), y0(m);
    x0[0] = 3.;
    x0[1] = 4.;
    y0    = f.Forward(0, x0);
    ok  &= NearEqual(y0[0] , x0[0]*x0[0]*x0[1], eps, eps);
    ok  &= f.size_order() == 1;

    // first order forward mode using notation in forward_one
    // X(t)           = x0 + x1 * t
    // Y(t) = F[X(t)] = y0 + y1 * t + o(t)
    Vector x1(n), y1(m);
    x1[0] = 1.;
    x1[1] = 0.;
    y1    = f.Forward(1, x1); // partial F w.r.t. x_0
    ok   &= NearEqual(y1[0] , 2.*x0[0]*x0[1], eps, eps);
    ok   &= f.size_order() == 2;

    // second order forward mode using notation in forward_order
    // X(t) =           x0 + x1 * t + x2 * t^2
    // Y(t) = F[X(t)] = y0 + y1 * t + y2 * t^2 + o(t^3)
    Vector x2(n), y2(m);
    x2[0]      = 0.;
    x2[1]      = 0.;
    y2         = f.Forward(2, x2);
    double F_00 = 2. * y2[0]; // second partial F w.r.t. x_0, x_0
    ok         &= NearEqual(F_00, 2.*x0[1], eps, eps);
    ok         &= f.size_order() == 3;

    return ok;
}
} // End empty namespace
# include <vector>
# include <valarray>
bool Forward(void)
{   bool ok = true;
    // Run with Vector equal to three different cases
    // all of which are Simple Vectors with elements of type double.
    ok &= ForwardCases< CppAD::vector  <double> >();
    ok &= ForwardCases< std::vector    <double> >();
    ok &= ForwardCases< std::valarray  <double> >();
    return ok;
}
// END C++
