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
$begin qp_interior.cpp$$
$spell
    qp
$$

$section abs_normal qp_interior: Example and Test$$

$head Problem$$
Our original problem is
$latex \[
    \R{minimize} \; | u - 1| \; \R{w.r.t} \; u \in \B{R}
\] $$
We reformulate this as the following problem
$latex \[
\begin{array}{rlr}
    \R{minimize}      & v             & \R{w.r.t} \; (u,v) \in \B{R}^2 \\
    \R{subject \; to} &  u - 1 \leq v \\
                      &  1 - u \leq v
\end{array}
\] $$
This is equivalent to
$latex \[
\begin{array}{rlr}
    \R{minimize}
    & (0, 1) \cdot (u, v)^T  & \R{w.r.t} \; (u,v) \in \B{R}^2 \\
    \R{subject \; to}
    &
    \left( \begin{array}{cc} 1 & -1 \\ -1 & -1 \end{array} \right)
    \left( \begin{array}{c} u \\ v \end{array} \right)
    +
    \left( \begin{array}{c} -1 \\ 1 \end{array} \right)
    \leq
    0
\end{array}
\] $$
which is in the form expected by $cref qp_interior$$.


$head Source$$
$srcthisfile%
    0%// BEGIN C++%// END C++%
1%$$

$end
*/
// BEGIN C++
# include <limits>
# include <cppad/utility/vector.hpp>
# include "qp_interior.hpp"

bool qp_interior(void)
{   bool ok = true;
    typedef CppAD::vector<double> vector;
    //
    size_t n = 2;
    size_t m = 2;
    vector C(m*n), c(m), G(n*n), g(n), xin(n), xout(n), yout(m), sout(m);
    C[ 0 * n + 0 ] =  1.0; // C(0,0)
    C[ 0 * n + 1 ] = -1.0; // C(0,1)
    C[ 1 * n + 0 ] = -1.0; // C(1,0)
    C[ 1 * n + 1 ] = -1.0; // C(1,1)
    //
    c[0]           = -1.0;
    c[1]           =  1.0;
    //
    g[0]           =  0.0;
    g[1]           =  1.0;
    //
    // G = 0
    for(size_t i = 0; i < n * n; i++)
        G[i] = 0.0;
    //
    // If (u, v) = (0,2), C * (u, v) + c = (-2,-2)^T + (1,-1)^T < 0
    // Hence (0, 2) is feasible.
    xin[0] = 0.0;
    xin[1] = 2.0;
    //
    double epsilon = 99.0 * std::numeric_limits<double>::epsilon();
    size_t maxitr  = 10;
    size_t level   = 0;
    //
    ok &= CppAD::qp_interior(
        level, c, C, g, G, epsilon, maxitr, xin, xout, yout, sout
    );
    //
    // check optimal value for u
    ok &= std::fabs( xout[0] - 1.0 ) < epsilon;
    // check optimal value for v
    ok &= std::fabs( xout[1] ) < epsilon;
    //
    return ok;
}
// END C++
