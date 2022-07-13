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
$begin abs_min_quad.cpp$$

$section abs_min_quad: Example and Test$$

$head Purpose$$
The function
$latex f : \B{R}^3 \rightarrow \B{R}$$ defined by
$latex \[
f( x_0, x_1  )
=
( x_0^2 + x_1^2 ) / 2 +  | x_0 - 5 | + | x_1 + 5 |
\] $$
For this case, the $cref abs_min_quad$$ object should be equal
to the function itself.
In addition, the function is convex and
$cref abs_min_quad$$ should find its global minimizer.
The minimizer of this function is
$latex x_0 = 1$$, $latex x_1 = -1$$.

$head Source$$
$srcthisfile%
    0%// BEGIN C++%// END C++%
1%$$

$end
-------------------------------------------------------------------------------
*/
// BEGIN C++
# include <cppad/cppad.hpp>
# include "abs_min_quad.hpp"

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
bool abs_min_quad(void)
{   bool ok = true;
    //
    using CppAD::AD;
    using CppAD::ADFun;
    //
    typedef CPPAD_TESTVECTOR(size_t)       s_vector;
    typedef CPPAD_TESTVECTOR(double)       d_vector;
    typedef CPPAD_TESTVECTOR( AD<double> ) ad_vector;
    //
    size_t level = 0;     // level of tracing
    size_t n     = 2;     // size of x
    size_t m     = 1;     // size of y
    size_t s     = 2 ;    // number of data points and absolute values
    //
    // record the function f(x)
    ad_vector ad_x(n), ad_y(m);
    for(size_t j = 0; j < n; j++)
        ad_x[j] = double(j + 1);
    Independent( ad_x );
    AD<double> sum = 0.0;
    sum += ad_x[0] * ad_x[0] / 2.0 + abs( ad_x[0] - 5 );
    sum += ad_x[1] * ad_x[1] / 2.0 + abs( ad_x[1] + 5 );
    ad_y[0] = sum;
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
    // Choose the point x_hat = 0
    d_vector x_hat(n);
    for(size_t j = 0; j < n; j++)
        x_hat[j] = 0.0;

    // value of a_hat = a(x_hat)
    d_vector a_hat = a.Forward(0, x_hat);

    // (x_hat, a_hat)
    d_vector xu_hat = join(x_hat, a_hat);

    // value of g[ x_hat, a_hat ]
    d_vector g_hat = g.Forward(0, xu_hat);

    // Jacobian of g[ x_hat, a_hat ]
    d_vector g_jac = g.Jacobian(xu_hat);

    // trust region bound
    d_vector bound(n);
    for(size_t j = 0; j < n; j++)
        bound[j] = 10.0;

    // convergence criteria
    d_vector epsilon(2);
    double eps99 = 99.0 * std::numeric_limits<double>::epsilon();
    epsilon[0]   = eps99;
    epsilon[1]   = eps99;

    // maximum number of iterations
    s_vector maxitr(2);
    maxitr[0] = 10; // maximum number of abs_min_quad iterations
    maxitr[1] = 35; // maximum number of qp_interior iterations

    // set Hessian equal to identity matrix I
    d_vector hessian(n * n);
    for(size_t i = 0; i < n; i++)
    {   for(size_t j = 0; j < n; j++)
            hessian[i * n + j] = 0.0;
        hessian[i * n + i] = 1.0;
    }

    // minimize the approxiamtion for f (which is equal to f for this case)
    d_vector delta_x(n);
    ok &= CppAD::abs_min_quad(
        level, n, m, s,
        g_hat, g_jac, hessian, bound, epsilon, maxitr, delta_x
    );

    // check that the solution
    ok &= CppAD::NearEqual( delta_x[0], +1.0, eps99, eps99 );
    ok &= CppAD::NearEqual( delta_x[1], -1.0, eps99, eps99 );

    return ok;
}
// END C++
