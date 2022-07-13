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
$begin abs_eval.cpp$$
$spell
    eval
$$

$section abs_eval: Example and Test$$

$head Purpose$$
The function
$latex f : \B{R}^3 \rightarrow \B{R}$$ defined by
$latex \[
    f( x_0, x_1, x_2  ) = | x_0 + x_1 | + | x_1 + x_2 |
\] $$
is affine, except for its absolute value terms.
For this case, the abs_normal approximation should be equal
to the function itself.

$head Source$$
$srcthisfile%
    0%// BEGIN C++%// END C++%
1%$$

$end
-------------------------------------------------------------------------------
*/
// BEGIN C++
# include <cppad/cppad.hpp>
# include "abs_eval.hpp"

namespace {
    CPPAD_TESTVECTOR(double) join(
        const CPPAD_TESTVECTOR(double)& x ,
        const CPPAD_TESTVECTOR(double)& u )
    {   size_t n = x.size();
        size_t s = u.size();
        CPPAD_TESTVECTOR(double) xu(n + s);
        for(size_t j = 0; j < n; j++)
            xu[j] = x[j];
        for(size_t j = 0; j < s; j++)
            xu[n + j] = u[j];
        return xu;
    }
}
bool abs_eval(void)
{   bool ok = true;
    //
    using CppAD::AD;
    using CppAD::ADFun;
    //
    typedef CPPAD_TESTVECTOR(double)       d_vector;
    typedef CPPAD_TESTVECTOR( AD<double> ) ad_vector;
    //
    double eps99 = 99.0 * std::numeric_limits<double>::epsilon();
    //
    size_t n = 3; // size of x
    size_t m = 1; // size of y
    size_t s = 2; // number of absolute value terms
    //
    // record the function f(x)
    ad_vector ad_x(n), ad_y(m);
    for(size_t j = 0; j < n; j++)
        ad_x[j] = double(j + 1);
    Independent( ad_x );
    // for this example, we ensure first absolute value is | x_0 + x_1 |
    AD<double> ad_0 = abs( ad_x[0] + ad_x[1] );
    // and second absolute value is | x_1 + x_2 |
    AD<double> ad_1 = abs( ad_x[1] + ad_x[2] );
    ad_y[0]         = ad_0 + ad_1;
    ADFun<double> f(ad_x, ad_y);

    // create its abs_normal representation in g, a
    ADFun<double> g, a;
    f.abs_normal_fun(g, a);

    // check dimension of domain and range space for g
    ok &= g.Domain() == n + s;
    ok &= g.Range()  == m + s;

    // check dimension of domain and range space for a
    ok &= a.Domain() == n;
    ok &= a.Range()  == s;

    // --------------------------------------------------------------------
    // Choose a point x_hat
    d_vector x_hat(n);
    for(size_t j = 0; j < n; j++)
        x_hat[j] = double(j - 1);

    // value of a_hat = a(x_hat)
    d_vector a_hat = a.Forward(0, x_hat);

    // (x_hat, a_hat)
    d_vector xu_hat = join(x_hat, a_hat);

    // value of g[ x_hat, a_hat ]
    d_vector g_hat = g.Forward(0, xu_hat);

    // Jacobian of g[ x_hat, a_hat ]
    d_vector g_jac = g.Jacobian(xu_hat);

    // value of delta_x
    d_vector delta_x(n);
    delta_x[0] =  1.0;
    delta_x[1] = -2.0;
    delta_x[2] = +2.0;

    // value of x
    d_vector x(n);
    for(size_t j = 0; j < n; j++)
        x[j] = x_hat[j] + delta_x[j];

    // value of f(x)
    d_vector y = f.Forward(0, x);

    // value of g_tilde
    d_vector g_tilde = CppAD::abs_eval(n, m, s, g_hat, g_jac, delta_x);

    // should be equal because f is affine, except for abs terms
    ok &= CppAD::NearEqual(y[0], g_tilde[0], eps99, eps99);

    return ok;
}
// END C++
