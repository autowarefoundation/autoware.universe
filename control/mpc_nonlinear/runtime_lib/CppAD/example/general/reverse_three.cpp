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
$begin reverse_three.cpp$$
$spell
    Taylor
    Cpp
$$

$section Third Order Reverse Mode: Example and Test$$


$head Taylor Coefficients$$
$latex \[
\begin{array}{rcl}
    X(t) & = & x^{(0)} + x^{(1)} t + x^{(2)} t^2
    \\
    X^{(1)} (t) & = &  x^{(1)} + 2 x^{(2)} t
    \\
    X^{(2)} (t) & = &   2 x^{(2)}
\end{array}
\] $$
Thus, we need to be careful to properly account for the fact that
$latex X^{(2)} (0) = 2 x^{(2)}$$
(and similarly $latex Y^{(2)} (0) = 2 y^{(2)}$$).

$srcthisfile%0%// BEGIN C++%// END C++%1%$$

$end
*/
// BEGIN C++
# include <cppad/cppad.hpp>
namespace { // ----------------------------------------------------------
// define the template function cases<Vector> in empty namespace
template <class Vector>
bool cases(void)
{   bool ok    = true;
    double eps = 10. * CppAD::numeric_limits<double>::epsilon();
    using CppAD::AD;
    using CppAD::NearEqual;

    // domain space vector
    size_t n = 2;
    CPPAD_TESTVECTOR(AD<double>) X(n);
    X[0] = 0.;
    X[1] = 1.;

    // declare independent variables and start recording
    CppAD::Independent(X);

    // range space vector
    size_t m = 1;
    CPPAD_TESTVECTOR(AD<double>) Y(m);
    Y[0] = X[0] * X[1];

    // create f : X -> Y and stop recording
    CppAD::ADFun<double> f(X, Y);

    // define x^0 and compute y^0 using user zero order forward
    Vector x0(n), y0(m);
    x0[0]    = 2.;
    x0[1]    = 3.;
    y0       = f.Forward(0, x0);

    // y^0 = F(x^0)
    double check;
    check    =  x0[0] * x0[1];
    ok      &= NearEqual(y0[0] , check, eps, eps);

    // define x^1 and compute y^1 using first order forward mode
    Vector x1(n), y1(m);
    x1[0] = 4.;
    x1[1] = 5.;
    y1    = f.Forward(1, x1);

    // Y^1 (x) = partial_t F( x^0 + x^1 * t )
    // y^1     = Y^1 (0)
    check = x1[0] * x0[1] + x0[0] * x1[1];
    ok   &= NearEqual(y1[0], check, eps, eps);

    // define x^2 and compute y^2 using second order forward mode
    Vector x2(n), y2(m);
    x2[0] = 6.;
    x2[1] = 7.;
    y2    = f.Forward(2, x2);

    // Y^2 (x) = partial_tt F( x^0 + x^1 * t + x^2 * t^2 )
    // y^2     = (1/2) *  Y^2 (0)
    check  = x2[0] * x0[1] + x1[0] * x1[1] + x0[0] * x2[1];
    ok    &= NearEqual(y2[0], check, eps, eps);

    // W(x)  = Y^0 (x) + 2 * Y^1 (x) + 3 * (1/2) * Y^2 (x)
    size_t p = 3;
    Vector dw(n*p), w(m*p);
    w[0] = 1.;
    w[1] = 2.;
    w[2] = 3.;
    dw   = f.Reverse(p, w);

    // check partial w.r.t x^0_0 of W(x)
    check = x0[1] + 2. * x1[1] + 3. * x2[1];
    ok   &= NearEqual(dw[0*p+0], check, eps, eps);

    // check partial w.r.t x^0_1 of W(x)
    check = x0[0] + 2. * x1[0] + 3. * x2[0];
    ok   &= NearEqual(dw[1*p+0], check, eps, eps);

    // check partial w.r.t x^1_0 of W(x)
    check = 2. * x0[1] + 3. * x1[1];
    ok   &= NearEqual(dw[0*p+1], check, eps, eps);

    // check partial w.r.t x^1_1 of W(x)
    check = 2. * x0[0] + 3. * x1[0];
    ok   &= NearEqual(dw[1*p+1], check, eps, eps);

    // check partial w.r.t x^2_0 of W(x)
    check = 3. * x0[1];
    ok   &= NearEqual(dw[0*p+2], check, eps, eps);

    // check partial w.r.t x^2_1 of W(x)
    check = 3. * x0[0];
    ok   &= NearEqual(dw[1*p+2], check, eps, eps);

    return ok;
}
} // End empty namespace
# include <vector>
# include <valarray>
bool reverse_three(void)
{   bool ok = true;
    ok &= cases< CppAD::vector  <double> >();
    ok &= cases< std::vector    <double> >();
    ok &= cases< std::valarray  <double> >();
    return ok;
}
// END C++
