/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-20 Bradley M. Bell

CppAD is distributed under the terms of the
             Eclipse Public License Version 2.0.

This Source Code may also be made available under the following
Secondary License when the conditions for such availability set forth
in the Eclipse Public License, Version 2.0 are satisfied:
      GNU General Public License, Version 2.0 or later.
---------------------------------------------------------------------------- */
# include <iostream>
# include <cppad/cppad.hpp>

// Test multiple level conditional skip where value of comparison is
// uncertain during forward mode base Base value can be a variable.
bool mul_cskip(void)
{   bool ok = true;
    using namespace CppAD;
    using CppAD::vector;

    typedef AD<double>  a1type;
    typedef AD<a1type>  a2type;

    size_t n = 2;
    size_t m = 1;
    vector<double> x(n), y(m);
    x[0] = 0.0;
    x[1] = 1.0;

    // start recording a2type operations
    vector<a2type> a2x(n), a2y(m);
    for (size_t j = 0; j < n; j++)
        a2x[j] = a2type( a1type(x[j]) );
    Independent(a2x);

    // a1f(x) = x_0 * x_1 if x[0] == 1
    //         0.0       otherwise
    a2type a2zero = a2type(0.0);
    a2type a2one  = a2type(1.0);
    a2type a2p    = a2x[0] * a2x[1];
    a2y[0]        = CondExpEq(a2x[0], a2one, a2p, a2zero);
    ADFun<a1type> a1f(a2x, a2y);

    // Optimization will check to see if we can skip part of conditional
    // expression that is not used.
    a1f.optimize();

    // f(x) = x_0 * x_1 if x[0] == 1
    //        0.0       otherwise
    vector<a1type> a1x(n), a1y(m);
    for (size_t j = 0; j < n; j++)
        a1x[j] = a1type(x[j]);
    Independent(a1x);
    a1y = a1f.Forward(0, a1x);
    CppAD::ADFun<double> f(a1x, a1y);

    // check case where x[0] == 1
    x[0] = 1.0;
    x[1] = 2.0;
    y = f.Forward(0, x);
    ok &= y[0] == x[1];

    // check case where x[0] != 1
    x[0] = 3.0;
    x[1] = 2.0;
    y = f.Forward(0, x);
    ok &= y[0] == 0.0;

    return ok;
}
