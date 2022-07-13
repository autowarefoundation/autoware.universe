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
$begin qp_box.cpp$$
$spell
    qp
$$

$section abs_normal qp_box: Example and Test$$

$head Problem$$
Our original problem is
$latex \[
\begin{array}{rl}
\R{minimize}      & x_0 - x_1 \; \R{w.r.t} \; x \in \B{R}^2 \\
\R{subject \; to} & -2 \leq x_0 \leq +2 \; \R{and} \; -2 \leq x_1 \leq +2
\end{array}
\] $$

$head Source$$
$srcthisfile%
    0%// BEGIN C++%// END C++%
1%$$

$end
*/
// BEGIN C++
# include <limits>
# include <cppad/utility/vector.hpp>
# include "qp_box.hpp"

bool qp_box(void)
{   bool ok = true;
    typedef CppAD::vector<double> vector;
    //
    size_t n = 2;
    size_t m = 0;
    vector a(n), b(n), c(m), C(m), g(n), G(n*n), xin(n), xout(n);
    a[0] = -2.0;
    a[1] = -2.0;
    b[0] = +2.0;
    b[1] = +2.0;
    g[0] = +1.0;
    g[1] = -1.0;
    for(size_t i = 0; i < n * n; i++)
        G[i] = 0.0;
    //
    // (0, 0) is feasible.
    xin[0] = 0.0;
    xin[1] = 0.0;
    //
    size_t level   = 0;
    double epsilon = 99.0 * std::numeric_limits<double>::epsilon();
    size_t maxitr  = 20;
    //
    ok &= CppAD::qp_box(
        level, a, b, c, C, g, G, epsilon, maxitr, xin, xout
    );
    //
    // check optimal value for x
    ok &= std::fabs( xout[0] + 2.0 ) < epsilon;
    ok &= std::fabs( xout[1] - 2.0 ) < epsilon;
    //
    return ok;
}
// END C++
