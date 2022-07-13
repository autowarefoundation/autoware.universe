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
$begin chkpoint_two_get_started.cpp$$
$spell
    checkpointing
    Taylor
$$

$section Get Started Checkpointing: Example and Test$$

$head Purpose$$
Break a large computation into pieces and only store values at the
interface of the pieces.
In actual applications, there may many uses of each function
and many more functions.

$head f$$
The function $latex f : \B{R}^2 \rightarrow \B{R}^2$$
is defined by
$latex \[
    f(y) = \left( \begin{array}{c}
        y_0 + y_0 + y_0
        \\
        y_1 + y_1 + y_1
    \end{array} \right)
\] $$


$head g$$
The function $latex g : \B{R}^2 \rightarrow \B{R}^2$$
defined by
$latex \[
    g(x) = \left( \begin{array}{c}
        x_0 \cdot x_0 \cdot x_0
        \\
        x_1 \cdot x_1 \cdot x_1
    \end{array} \right)
\] $$

$head f[g(x)]$$
The function $latex f[g(x)]$$ is given by
$latex \[
f[g(x)]
=
f \left[ \begin{array}{c}
    x_0^3 \\
    x_1^3
\end{array} \right]
=
\left[ \begin{array}{c}
    3 x_0^3 \\
    3 x_1^3
\end{array} \right]
\] $$

$head Source Code$$
$srcthisfile%0%// BEGIN C++%// END C++%1%$$

$end
*/
// BEGIN C++

# include <cppad/cppad.hpp>

namespace {
    using CppAD::AD;
    typedef CPPAD_TESTVECTOR(AD<double>)            ADVector;

    void f_algo(const ADVector& y, ADVector& z)
    {   z[0] = 0.0;
        z[1] = 0.0;
        for(size_t k = 0; k < 3; k++)
        {   z[0] += y[0];
            z[1] += y[1];
        }
        return;
    }
    void g_algo(const ADVector& x, ADVector& y)
    {   y[0] = 1.0;
        y[1] = 1.0;
        for(size_t k = 0; k < 3; k++)
        {   y[0] *= x[0];
            y[1] *= x[1];
        }
        return;
    }
}
bool get_started(void)
{   bool ok = true;
    using CppAD::NearEqual;
    double eps99 = 99.0 * std::numeric_limits<double>::epsilon();

    // AD vectors holding x, y, and z values
    size_t nx = 2, ny = 2, nz = 2;
    ADVector ax(nx), ay(ny), az(nz);

    // record the function g_fun(x)
    for(size_t j = 0; j < nx; j++)
        ax[j] = double(j + 1);
    Independent(ax);
    g_algo(ax, ay);
    CppAD::ADFun<double> g_fun(ax, ay);

    // record the function f_fun(y)
    Independent(ay);
    f_algo(ay, az);
    CppAD::ADFun<double> f_fun(ay, az);

    // create checkpoint versions of f and g
    bool internal_bool    = false;
    bool use_hes_sparsity = false;
    bool use_base2ad      = false;
    bool use_in_parallel  = false;
    CppAD::chkpoint_two<double> f_chk( f_fun, "f_chk",
        internal_bool, use_hes_sparsity, use_base2ad, use_in_parallel
    );
    CppAD::chkpoint_two<double> g_chk( g_fun, "g_chk",
        internal_bool, use_hes_sparsity, use_base2ad, use_in_parallel
    );

    // Record a version of z = f[g(x)] using checkpointing
    Independent(ax);
    g_chk(ax, ay);
    f_chk(ay, az);
    CppAD::ADFun<double> fg(ax, az);

    // zero order forward mode
    CPPAD_TESTVECTOR(double) x(nx), z(nz);
    for(size_t j = 0; j < nx; j++)
        x[j] = 1.0 / double(1 + j);
    z = fg.Forward(0, x);
    for(size_t i = 0; i < nz; i++)
    {   double check = 3.0 * x[i] * x[i] * x[i];
        ok &= NearEqual(z[i], check, eps99, eps99);
    }

    // optimize fg and check that results do not change
    fg.optimize();
    for(size_t i = 0; i < nz; i++)
    {   double check = 3.0 * x[i] * x[i] * x[i];
        ok &= NearEqual(z[i], check, eps99, eps99);
    }
    //
    return ok;
}
// END C++
