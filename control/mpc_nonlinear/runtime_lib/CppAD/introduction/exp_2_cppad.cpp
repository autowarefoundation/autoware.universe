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
$begin exp_2_cppad$$
$spell
    Taylor
    coef
    resize
    cppad.hpp
    cmath
    fabs
    bool
    exp_2_cppad
    dx
    dy
    dw
    endl
    hpp
    http
    org
    std
    www
    CppAD
    apx
$$

$section exp_2: CppAD Forward and Reverse Sweeps$$.

$head Purpose$$
Use CppAD forward and reverse modes to compute the
partial derivative with respect to $latex x$$,
at the point $latex x = .5$$,
of the function
$codei%
    exp_2(%x%)
%$$
as defined by the $cref exp_2.hpp$$ include file.

$head Exercises$$
$list number$$
Create and test a modified version of the routine below that computes
the same order derivatives with respect to $latex x$$,
at the point $latex x = .1$$
of the function
$codei%
    exp_2(%x%)
%$$
$lnext
Create a routine called
$codei%
    exp_3(%x%)
%$$
that evaluates the function
$latex \[
    f(x) = 1 + x^2 / 2 + x^3 / 6
\] $$
Test a modified version of the routine below that computes
the derivative of $latex f(x)$$
at the point $latex x = .5$$.
$lend
$srccode%cpp% */

# include <cppad/cppad.hpp>  // http://www.coin-or.org/CppAD/
# include "exp_2.hpp"        // second order exponential approximation
bool exp_2_cppad(void)
{   bool ok = true;
    using CppAD::AD;
    using CppAD::vector;    // can use any simple vector template class
    using CppAD::NearEqual; // checks if values are nearly equal

    // domain space vector
    size_t n = 1; // dimension of the domain space
    vector< AD<double> > X(n);
    X[0] = .5;    // value of x for this operation sequence

    // declare independent variables and start recording operation sequence
    CppAD::Independent(X);

    // evaluate our exponential approximation
    AD<double> x   = X[0];
    AD<double> apx = exp_2(x);

    // range space vector
    size_t m = 1;  // dimension of the range space
    vector< AD<double> > Y(m);
    Y[0] = apx;    // variable that represents only range space component

    // Create f: X -> Y corresponding to this operation sequence
    // and stop recording. This also executes a zero order forward
    // sweep using values in X for x.
    CppAD::ADFun<double> f(X, Y);

    // first order forward sweep that computes
    // partial of exp_2(x) with respect to x
    vector<double> dx(n);  // differential in domain space
    vector<double> dy(m);  // differential in range space
    dx[0] = 1.;            // direction for partial derivative
    dy    = f.Forward(1, dx);
    double check = 1.5;
    ok   &= NearEqual(dy[0], check, 1e-10, 1e-10);

    // first order reverse sweep that computes the derivative
    vector<double>  w(m);   // weights for components of the range
    vector<double> dw(n);   // derivative of the weighted function
    w[0] = 1.;              // there is only one weight
    dw   = f.Reverse(1, w); // derivative of w[0] * exp_2(x)
    check = 1.5;            // partial of exp_2(x) with respect to x
    ok   &= NearEqual(dw[0], check, 1e-10, 1e-10);

    // second order forward sweep that computes
    // second partial of exp_2(x) with respect to x
    vector<double> x2(n);     // second order Taylor coefficients
    vector<double> y2(m);
    x2[0] = 0.;               // evaluate second partial .w.r.t. x
    y2    = f.Forward(2, x2);
    check = 0.5 * 1.;         // Taylor coef is 1/2 second derivative
    ok   &= NearEqual(y2[0], check, 1e-10, 1e-10);

    // second order reverse sweep that computes
    // derivative of partial of exp_2(x) w.r.t. x
    dw.resize(2 * n);         // space for first and second derivatives
    dw    = f.Reverse(2, w);
    check = 1.;               // result should be second derivative
    ok   &= NearEqual(dw[0*2+1], check, 1e-10, 1e-10);

    return ok;
}

/* %$$
$end
*/
