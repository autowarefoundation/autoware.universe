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
$begin number_skip.cpp$$
$spell
    Taylor
$$

$section Number of Variables That Can be Skipped: Example and Test$$


$srcthisfile%0%// BEGIN C++%// END C++%1%$$

$end
*/
// BEGIN C++
# include <cppad/cppad.hpp>
bool number_skip(void)
{   bool ok = true;
    using CppAD::AD;

    // independent variable vector
    CppAD::vector< AD<double> > ax(2);
    ax[0] = 0.;
    ax[1] = 1.;
    Independent(ax);

    // Use a conditional expression
    CppAD::vector< AD<double> > ay(1);

    // variable that gets optimized out
    AD<double> az = ax[0] * ax[0];


    // conditional expression
    ay[0] = CondExpLt(ax[0], ax[1], ax[0] + ax[1], ax[0] - ax[1]);

    // create function object F : x -> ay
    CppAD::ADFun<double> f;
    f.Dependent(ax, ay);

    // use zero order to evaluate F[ (3, 4) ]
    CppAD::vector<double>  x( f.Domain() );
    CppAD::vector<double>  y( f.Range() );
    x[0]    = 3.;
    x[1]    = 4.;
    y   = f.Forward(0, x);
    ok &= (y[0] == x[0] + x[1]);

    // before call to optimize
    ok &= f.number_skip() == 0;
    size_t n_var = f.size_var();

    // now optimize the operation sequence
    f.optimize();

    // after optimize, check forward mode result
    x[0]    = 4.;
    x[1]    = 3.;
    y   = f.Forward(0, x);
    ok &= (y[0] == x[0] - x[1]);

    // after optimize, check amount of optimization
    ok &= f.size_var() == n_var - 1;
    ok &= f.number_skip() == 1;

    return ok;
}

// END C++
