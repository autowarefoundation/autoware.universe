/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-17 Bradley M. Bell

CppAD is distributed under the terms of the
             Eclipse Public License Version 2.0.

This Source Code may also be made available under the following
Secondary License when the conditions for such availability set forth
in the Eclipse Public License, Version 2.0 are satisfied:
      GNU General Public License, Version 2.0 or later.
---------------------------------------------------------------------------- */

/*
Test the use of the parameters in VecAD element assignments
*/

# include <cppad/cppad.hpp>

typedef CppAD::AD<double>      ADdouble;
typedef CppAD::AD< ADdouble > ADDdouble;

bool VecADPar(void)
{
    using namespace CppAD;

    bool ok = true;

    CPPAD_TESTVECTOR( ADdouble ) x(2);
    x[0] = 0;
    x[1] = 0;
    Independent(x);

    CPPAD_TESTVECTOR( ADDdouble ) y(1);
    y[0] = 1;
    Independent(y);

    VecAD< ADdouble > v(2);
    ADDdouble zero(0);
    ADDdouble one(1);
    v[zero] = x[0]; // these two parameter values are equal,
    v[one]  = x[1]; // but they are not identically equal

    CPPAD_TESTVECTOR( ADDdouble ) z(1);
    z[0] = v[zero] + v[one];

    // f(y) = x[0] + x[1]
    ADFun< ADdouble > f(y, z);
    CPPAD_TESTVECTOR( ADdouble ) a( f.Domain() );
    CPPAD_TESTVECTOR( ADdouble ) b( f.Range() );

    // fy = f(y) = x[0] + x[1]
    a[0]  = 0.;
    b     = f.Forward(0, a);

    // check value of f
    ok &= b[0] == (x[0] + x[1]);

    // g(x) = x[0] + x[1];
    ADFun<double> g(x, b);
    CPPAD_TESTVECTOR( double ) c( g.Domain() );
    CPPAD_TESTVECTOR( double ) d( g.Range() );

    // d = g(1, 2)
    c[0] = 1.;   // these tow values are not equal and correspond
    c[1] = 2.;   // to replacements for the equal parameter values above
    d = g.Forward(0, c);

    // check function value
    ok &= (d[0] == c[0] + c[1]);

    return ok;
}
