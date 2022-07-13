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
Old example now used just for validation testing.
*/
# include <cppad/cppad.hpp>

bool log10(void)
{   bool ok = true;
    using CppAD::log10;
    using CppAD::log;
    using namespace CppAD;
    double eps99 = 99.0 * std::numeric_limits<double>::epsilon();

    // independent variable vector, indices, values, and declaration
    CPPAD_TESTVECTOR(AD<double>) U(1);
    size_t s = 0;
    U[s]     = 10.;
    Independent(U);

    // dependent variable vector, indices, and values
    CPPAD_TESTVECTOR(AD<double>) Z(2);
    size_t x = 0;
    size_t y = 1;
    Z[x]     = log10(U[s]);
    Z[y]     = log10(Z[x]);

    // define f : U -> Z and vectors for derivative calculations
    ADFun<double> f(U, Z);
    CPPAD_TESTVECTOR(double) v( f.Domain() );
    CPPAD_TESTVECTOR(double) w( f.Range() );

    // check values
    ok &= NearEqual(Z[x] , 1.,  eps99 , eps99);
    ok &= NearEqual(Z[y] , 0.,  eps99 , eps99);

    // forward computation of partials w.r.t. s
    double l10 = log(10.);
    v[s] = 1.;
    w    = f.Forward(1, v);
    ok &= NearEqual(w[x], 1./(U[s]*l10)         , eps99 , eps99); // dx/ds
    ok &= NearEqual(w[y], 1./(U[s]*Z[x]*l10*l10), eps99 , eps99); // dy/ds

    // reverse computation of partials of y
    w[x] = 0.;
    w[y] = 1.;
    v    = f.Reverse(1,w);
    ok &= NearEqual(v[s], 1./(U[s]*Z[x]*l10*l10), eps99 , eps99); // dy/ds

    return ok;
}
