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
$begin lp_box.cpp$$
$spell
    lp
$$

$section abs_normal lp_box: Example and Test$$

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
# include "lp_box.hpp"

bool lp_box(void)
{   bool ok = true;
    typedef CppAD::vector<double> vector;
    double eps99 = 99.0 * std::numeric_limits<double>::epsilon();
    //
    size_t n = 2;
    size_t m = 0;
    vector A(m), b(m), c(n), d(n), xout(n);
    c[0] = +1.0;
    c[1] = -1.0;
    //
    d[0] = +2.0;
    d[1] = +2.0;
    //
    size_t level   = 0;
    size_t maxitr  = 20;
    //
    ok &= CppAD::lp_box(level, A, b, c, d, maxitr, xout);
    //
    // check optimal value for x
    ok &= std::fabs( xout[0] + 2.0 ) < eps99;
    ok &= std::fabs( xout[1] - 2.0 ) < eps99;
    //
    return ok;
}
// END C++
