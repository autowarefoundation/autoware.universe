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
$begin simplex_method.cpp$$
$spell
    qp
$$

$section abs_normal simplex_method: Example and Test$$

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
We know that the value of $latex v$$ at the solution is greater than
or equal zero. Hence we can reformulate this problem as
$latex \[
\begin{array}{rlr}
\R{minimize}      & v             & \R{w.r.t} \; ( u_- , u_+ , v) \in \B{R}_+^3 \\
\R{subject \; to} & u_+ - u_- - 1  \leq v \\
                  &  1 - u_+ + u_- \leq v
\end{array}
\] $$
This is equivalent to
$latex \[
\begin{array}{rlr}
    \R{minimize}
    & (0, 0, 1) \cdot ( u_+, u_- , v)^T  & \R{w.r.t} \; (u,v) \in \B{R}_+^3 \\
\R{subject \; to}
    &
    \left( \begin{array}{ccc}
        +1 & -1 & -1 \\
        -1 & +1 & +1
    \end{array} \right)
    \left( \begin{array}{c} u_+ \\ u_- \\ v \end{array} \right)
    +
    \left( \begin{array}{c} -1 \\ 1 \end{array} \right)
    \leq
    0
\end{array}
\] $$
which is in the form expected by $cref simplex_method$$.


$head Source$$
$srcthisfile%
    0%// BEGIN C++%// END C++%
1%$$

$end
*/
// BEGIN C++
# include <limits>
# include <cppad/utility/vector.hpp>
# include "simplex_method.hpp"

bool simplex_method(void)
{   bool ok = true;
    typedef CppAD::vector<double> vector;
    double eps99 = 99.0 * std::numeric_limits<double>::epsilon();
    //
    size_t n = 3;
    size_t m = 2;
    vector A(m * n), b(m), c(n), xout(n);
    A[ 0 * n + 0 ] =  1.0; // A(0,0)
    A[ 0 * n + 1 ] = -1.0; // A(0,1)
    A[ 0 * n + 2 ] = -1.0; // A(0,2)
    //
    A[ 1 * n + 0 ] = -1.0; // A(1,0)
    A[ 1 * n + 1 ] = +1.0; // A(1,1)
    A[ 1 * n + 2 ] = -1.0; // A(1,2)
    //
    b[0]           = -1.0;
    b[1]           =  1.0;
    //
    c[0]           =  0.0;
    c[1]           =  0.0;
    c[2]           =  1.0;
    //
    size_t maxitr  = 10;
    size_t level   = 0;
    //
    ok &= CppAD::simplex_method(level, A, b, c,  maxitr, xout);
    //
    // check optimal value for u
    ok &= std::fabs( xout[0] - 1.0 ) < eps99;
    //
    // check optimal value for v
    ok &= std::fabs( xout[1] ) < eps99;
    //
    return ok;
}
// END C++
