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
$begin optimize_twice.cpp$$
$spell
    CppAD
$$

$section Optimizing Twice: Example and Test$$

$head Discussion$$
Before 2019-06-28, optimizing twice was not supported and would fail
if cumulative sum operators were present after the first optimization.
This is now supported but it is not expected to have much benefit.
If you find a case where it does have a benefit, please inform the CppAD
developers of this.

$srcthisfile%0%// BEGIN C++%// END C++%1%$$

$end
*/
// BEGIN C++
# include <cppad/cppad.hpp>
bool optimize_twice(void)
{   bool ok = true;
    using CppAD::AD;
    using CppAD::vector;

    size_t n = 3;
    vector< AD<double> > ax(n);
    for(size_t j = 0; j < n; ++j)
        ax[j] = 0.0;
    Independent(ax);
    //
    AD<double> asum = 0.0;
    for(size_t j = 0; j < n; ++j)
        asum += ax[j];
    //
    vector< AD<double> > ay(1);
    ay[0] = asum * asum;
    //
    // This method of recording the function does not do a 0 order forward
    CppAD::ADFun<double> f;
    f.Dependent(ax, ay);
    ok &= f.size_order() == 0;
    size_t size_var = f.size_var();
    //
    f.optimize(); // creates a cumulative sum operator
    ok &= f.size_var() <= size_var - (n - 2);
    size_var = f.size_var();
    //
    f.optimize(); // optimizes a function with a cumulative sum operator
    ok &= f.size_var() == size_var; // no benefit expected by second optimize
    //
    // zero order forward
    vector<double> x(n), y(1);
    double sum = 0.0;
    for(size_t j = 0; j < n; ++j)
    {   x[j]  = double(j + 1);
        sum  += x[j];
    }
    y     = f.Forward(0, x);
    double check = sum * sum;
    ok &= y[0] == check;
    //
    vector<double> w(1), dx(n);
    w[0] = 1.0;
    dx   = f.Reverse(1, w);
    //
    for(size_t j = 0; j < n; ++j)
        ok &= dx[j] == 2.0 * sum;
    //
    return ok;
}

// END C++
