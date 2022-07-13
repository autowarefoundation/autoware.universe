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
old example and test now only used for testing
*/
// BEGIN C++

# include <cppad/cppad.hpp>

bool Neg(void)
{   bool ok = true;

    using namespace CppAD;

    // independent variable vector, indices, values, and declaration
    CPPAD_TESTVECTOR(AD<double>) U(2);
    size_t s = 0;
    size_t t = 1;
    U[s]     = 3.;
    U[t]     = 4.;
    Independent(U);

    // dependent variable vector and indices
    CPPAD_TESTVECTOR(AD<double>) Z(1);
    size_t x = 0;

    // dependent variable values
    Z[x] = - U[t];   //  - AD<double>

    // create f: U -> Z and vectors used for derivative calculations
    ADFun<double> f(U, Z);
    CPPAD_TESTVECTOR(double) v( f.Domain() );
    CPPAD_TESTVECTOR(double) w( f.Range() );

    // check values
    ok &= ( Z[x] == -4. );

    // forward computation of partials w.r.t. s
    v[s] = 1.;
    v[t] = 0.;
    w    = f.Forward(1, v);
    ok &= ( w[x] == 0. );   // dx/ds

    // forward computation of partials w.r.t. t
    v[s] = 0.;
    v[t] = 1.;
    w    = f.Forward(1, v);
    ok &= ( w[x] == -1. );   // dx/dt

    // reverse computation of second partials of z
    CPPAD_TESTVECTOR(double) r( f.Domain() * 2 );
    w[x] = 1.;
    r    = f.Reverse(2, w);
    ok &= ( r[2 * s + 1] == 0. );  // d^2 x / (ds ds)
    ok &= ( r[2 * t + 1] == 0. );  // d^2 x / (ds dt)

    return ok;
}
// END C++
