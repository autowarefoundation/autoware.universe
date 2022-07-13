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
$begin abs_get_started.cpp$$

$section abs_normal Getting Started: Example and Test$$

$head Purpose$$
Creates an $cref/abs_normal/abs_normal_fun/$$
representation $latex g$$ for the function
$latex f : \B{R}^3 \rightarrow \B{R}$$ defined by
$latex \[
    f( x_0, x_1, x_2  ) = | x_0 + x_1 | + | x_1 + x_2 |
\] $$
The corresponding
$cref/g/abs_normal_fun/g/$$ $latex : \B{R}^5 \rightarrow \B{R}^3$$ is
given by
$latex \[
\begin{array}{rclrcl}
    g_0 ( x_0, x_1, x_2, u_0, u_1 ) & = & u_0 + u_1 & = & y_0 (x, u)
    \\
    g_1 ( x_0, x_1, x_2, u_0, u_1 ) & = & x_0 + x_1 & = & z_0 (x, u)
    \\
    g_1 ( x_0, x_1, x_2, u_0, u_1 ) & = & x_1 + x_2 & = & z_1 (x, u)
\end{array}
\] $$

$head Source$$
$srcthisfile%
    0%// BEGIN C++%// END C++%
1%$$

$end
-------------------------------------------------------------------------------
*/
// BEGIN C++
# include <cppad/cppad.hpp>
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
bool get_started(void)
{   bool ok = true;
    //
    using CppAD::AD;
    using CppAD::ADFun;
    //
    size_t n = 3; // size of x
    size_t m = 1; // size of y
    size_t s = 2; // size of u and z
    //
    // record the function f(x)
    CPPAD_TESTVECTOR( AD<double> ) ax(n), ay(m);
    for(size_t j = 0; j < n; j++)
        ax[j] = double(j + 1);
    Independent( ax );
    // for this example, we ensure first absolute value is | x_0 + x_1 |
    AD<double> a0 = abs( ax[0] + ax[1] );
    // and second absolute value is | x_1 + x_2 |
    AD<double> a1 = abs( ax[1] + ax[2] );
    ay[0]         = a0 + a1;
    ADFun<double> f(ax, ay);

    // create its abs_normal representation in g, a
    ADFun<double> g, a;
    f.abs_normal_fun(g, a);

    // check dimension of domain and range space for g
    ok &= g.Domain() == n + s;
    ok &= g.Range() == m + s;

    // check dimension of domain and range space for a
    ok &= a.Domain() == n;
    ok &= a.Range() == s;

    // --------------------------------------------------------------------
    // a(x) has all the operations used to compute f(x), but the sum of the
    // absolute values is not needed for a(x), so optimize it out.
    size_t n_op = f.size_op();
    ok         &= a.size_op() == n_op;
    a.optimize();
    ok         &= a.size_op() < n_op;

    // --------------------------------------------------------------------
    // zero order forward mode calculation using g(x, u)
    CPPAD_TESTVECTOR(double) x(n), u(s), xu(n+s), yz(m+s);
    for(size_t j = 0; j < n; j++)
        x[j] = double(j + 2);
    for(size_t j = 0; j < s; j++)
        u[j] = double(j + n + 2);
    xu = join(x, u);
    yz = g.Forward(0, xu);

    // check y_0(x, u)
    double y0 = u[0] + u[1];
    ok       &= y0 == yz[0];

    // check z_0 (x, u)
    double z0 = x[0] + x[1];
    ok       &= z0 == yz[1];

    // check z_1 (x, u)
    double z1 = x[1] + x[2];
    ok       &= z1 == yz[2];


    // --------------------------------------------------------------------
    // check that y(x, a(x) ) == f(x)
    CPPAD_TESTVECTOR(double) y(m);
    y  = f.Forward(0, x);  // y  = f(x)
    u  = a.Forward(0, x);  // u  = a(x)
    xu = join(x, u);       // xu = ( x, a(x) )
    yz = g.Forward(0, xu); // yz = ( y(x, a(x)), z(x, a(x)) )
    ok &= yz[0] == y[0];

    return ok;
}
// END C++
