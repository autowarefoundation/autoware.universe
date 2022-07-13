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
$begin chkpoint_two_dynamic.cpp$$
$spell

$$

$section Checkpoint Functions with Dynamic Parameters: Example and Test$$

$head Purpose$$
This example demonstrates using dynamic parameters with a
$cref chkpoint_two$$ function.

$head g(x)$$
For this example, the checkpoint function
$latex g : \B{R}^2 \rightarrow \B{R}^3$$ is defined by
$latex \[
    g(x)
    =
    \left( \begin{array}{c}
        x_0 \cdot p_0 \\
        x_0 \cdot x_ 0 \\
        x_1 \cdot x_ 0
    \end{array} \right)
\] $$
where $latex p_0$$ is a dynamic parameter in the definition of $latex g(x)$$

$head f(x)$$
The function $latex f(x) : \B{R}^2 \rightarrow \B{R}^3$$
is defined by $latex f(x) = q_0 \cdot g(x)$$ where $latex q_0$$
is a dynamic parameter in the definition of $latex f(x)$$.

$head Source$$
$srcthisfile%0%// BEGIN C++%// END C++%1%$$

$end
*/
// BEGIN C++
# include <cppad/cppad.hpp>
bool dynamic(void)
{   bool ok = true;
    using CppAD::AD;
    using CppAD::NearEqual;
    double eps99 = 99.0 * CppAD::numeric_limits<double>::epsilon();
    //
    //
    // record the function g(x)
    size_t nx = 2, ny = 3, np = 1, nq = 1;
    CPPAD_TESTVECTOR( AD<double> ) ax(nx), ay(ny), ap(np), aq(nq);
    for(size_t j = 0; j < nx; j++)
        ax[j] = double(j);
    ap[0] = 2.0;
    size_t abort_op_index = 0;
    bool   record_compare = true;
    Independent(ax, abort_op_index, record_compare, ap);
    ay[0] = ax[0] * ap[0];
    ay[1] = ax[0] * ax[0];
    ay[2] = ax[1] * ax[0];
    CppAD::ADFun<double> g_fun(ax, ay);
    //
    // make a checkpoint version of g
    std::string name             = "g(x)";
    bool        internal_bool    = true;
    bool        use_hes_sparsity = false;
    bool        use_base2ad      = false;
    bool        use_in_parallel  = false;
    CppAD::chkpoint_two<double> g_chk(g_fun, name,
        internal_bool, use_hes_sparsity, use_base2ad, use_in_parallel
    );
    //
    // record f(x) using checkpoint version of g(x)
    aq[0] = 3.0;
    Independent(ax, abort_op_index, record_compare, aq);
    g_chk(ax, ay);
    for(size_t i = 0; i < ny; ++i)
        ay[i] = aq[0] * ay[i];
    CppAD::ADFun<double> f_fun(ax, ay);
    //
    // vecotrs of doubles
    CPPAD_TESTVECTOR(double) x(nx), y(ny), p(np), q(nq);
    //
    // set dynamic parameters in g_chk
    p[0] = 4.0;
    g_chk.new_dynamic(p);
    //
    // set dynamic parmeters in f_fun
    q[0] = 5.0;
    f_fun.new_dynamic(q);
    //
    // evaluate f(x)
    for(size_t j = 0; j < nx; ++j)
        x[j] = 6.0 + double(j);
    y = f_fun.Forward(0, x);
    //
    // check result
    double check;
    check = q[0] * x[0] * p[0];
    ok   &= NearEqual(check, y[0], eps99, eps99);
    check = q[0] * x[0] * x[0];
    ok   &= NearEqual(check, y[1], eps99, eps99);
    check = q[0] * x[1] * x[0];
    ok   &= NearEqual(check, y[2], eps99, eps99);
    //
    return ok;
}
// END C++
