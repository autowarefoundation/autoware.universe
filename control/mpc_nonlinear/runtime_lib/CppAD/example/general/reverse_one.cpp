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
$begin reverse_one.cpp$$
$spell
    Cpp
$$

$section First Order Reverse Mode: Example and Test$$


$srcthisfile%0%// BEGIN C++%// END C++%1%$$

$end
*/
// BEGIN C++
# include <cppad/cppad.hpp>
namespace { // ----------------------------------------------------------
// define the template function reverse_one_cases<Vector> in empty namespace
template <class Vector>
bool reverse_one_cases(void)
{   bool ok = true;
    using CppAD::AD;
    using CppAD::NearEqual;
    double eps99 = 99.0 * std::numeric_limits<double>::epsilon();

    // domain space vector
    size_t n = 2;
    CPPAD_TESTVECTOR(AD<double>) ax(n);
    ax[0] = 0.;
    ax[1] = 1.;

    // declare independent variables and start recording
    CppAD::Independent(ax);

    // range space vector
    size_t m = 1;
    CPPAD_TESTVECTOR(AD<double>) ay(m);
    ay[0] = ax[0] * ax[0] * ax[1];

    // create f : x -> y and stop recording
    CppAD::ADFun<double> f(ax, ay);

    // use first order reverse mode to evaluate derivative of y[0]
    // and use the values in x for the independent variables.
    CPPAD_TESTVECTOR(double) w(m), dw(n);
    w[0] = 1.;
    dw   = f.Reverse(1, w);
    ok  &= NearEqual(dw[0] , 2.*ax[0]*ax[1], eps99, eps99);
    ok  &= NearEqual(dw[1] ,    ax[0]*ax[0], eps99, eps99);

    // use zero order forward mode to evaluate y at x = (3, 4)
    // and use the template parameter Vector for the vector type
    Vector x(n), y(m);
    x[0]    = 3.;
    x[1]    = 4.;
    y       = f.Forward(0, x);
    ok     &= NearEqual(y[0] , x[0]*x[0]*x[1], eps99, eps99);

    // use first order reverse mode to evaluate derivative of y[0]
    // and using the values in x for the independent variables.
    w[0] = 1.;
    dw   = f.Reverse(1, w);
    ok  &= NearEqual(dw[0] , 2.*x[0]*x[1], eps99, eps99);
    ok  &= NearEqual(dw[1] ,    x[0]*x[0], eps99, eps99);

    return ok;
}
} // End empty namespace
# include <vector>
# include <valarray>
bool reverse_one(void)
{   bool ok = true;
    // Run with Vector equal to three different cases
    // all of which are Simple Vectors with elements of type double.
    ok &= reverse_one_cases< CppAD::vector  <double> >();
    ok &= reverse_one_cases< std::vector    <double> >();
    ok &= reverse_one_cases< std::valarray  <double> >();
    return ok;
}
// END C++
