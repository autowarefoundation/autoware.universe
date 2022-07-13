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
Two old exp example now used just for validation testing.
*/
# include <cppad/cppad.hpp>

# include <cmath>

namespace { // BEGIN empty namespace

bool ExpTestOne(void)
{   bool ok = true;
    using CppAD::exp;
    using namespace CppAD;
    double eps99 = 99.0 * std::numeric_limits<double>::epsilon();

    // independent variable vector, indices, values, and declaration
    CPPAD_TESTVECTOR(AD<double>) U(1);
    size_t s = 0;
    U[s]     = 1.;
    Independent(U);

    // dependent variable vector, indices, and values
    CPPAD_TESTVECTOR(AD<double>) Z(2);
    size_t x = 0;
    size_t y = 1;
    Z[x]     = exp(U[s]);
    Z[y]     = exp(Z[x]);

    // define f : U -> Z and vectors for derivative calculations
    ADFun<double> f(U, Z);
    CPPAD_TESTVECTOR(double) v( f.Domain() );
    CPPAD_TESTVECTOR(double) w( f.Range() );

    // check values
    ok &= NearEqual(Z[x] , exp(1.),         eps99 , eps99);
    ok &= NearEqual(Z[y] , exp( exp(1.) ),  eps99 , eps99);

    // forward computation of partials w.r.t. s
    v[s] = 1.;
    w    = f.Forward(1, v);
    ok &= NearEqual(w[x], Z[x],            eps99 , eps99); // dx/ds
    ok &= NearEqual(w[y], Z[y] * Z[x],     eps99 , eps99); // dy/ds

    // reverse computation of partials of y
    w[x] = 0.;
    w[y] = 1.;
    v    = f.Reverse(1,w);
    ok &= NearEqual(v[s], Z[y] * Z[x],     eps99 , eps99); // dy/ds

    // forward computation of second partials w.r.t s
    v[s] = 1.;
    w    = f.Forward(1, v);
    v[s] = 0.;
    w    = f.Forward(2, v);
    ok &= NearEqual(       // d^2 y / (ds ds)
        2. * w[y] ,
        Z[y] * Z[x] * Z[x] + Z[y] * Z[x],
        eps99 ,
        eps99
    );

    // reverse computation of second partials of y
    CPPAD_TESTVECTOR(double) r( f.Domain() * 2 );
    w[x] = 0.;
    w[y] = 1.;
    r    = f.Reverse(2, w);
    ok &= NearEqual(      // d^2 y / (ds ds)
        r[2 * s + 1] ,
        Z[y] * Z[x] * Z[x] + Z[y] * Z[x],
        eps99 ,
        eps99
    );

    return ok;
}
bool ExpTestTwo(void)
{   bool ok = true;
    using CppAD::exp;
    using namespace CppAD;
    double eps99 = 99.0 * std::numeric_limits<double>::epsilon();

    // independent variable vector
    CPPAD_TESTVECTOR(AD<double>) U(1);
    U[0]     = 1.;
    Independent(U);

    // dependent variable vector
    CPPAD_TESTVECTOR(AD<double>) Z(1);
    Z[0] = exp(U[0]);

    // create f: U -> Z and vectors used for derivative calculations
    ADFun<double> f(U, Z);
    CPPAD_TESTVECTOR(double) v(1);
    CPPAD_TESTVECTOR(double) w(1);

    // check value
    double exp_u = exp( Value(U[0]) );
    ok &= NearEqual(exp_u, Value(Z[0]),  eps99 , eps99);

    // forward computation of partials w.r.t. u
    size_t j;
    size_t p     = 5;
    double jfac  = 1.;
    v[0]         = 1.;
    for(j = 1; j < p; j++)
    {   w     = f.Forward(j, v);
        jfac *= double(j);
        ok &= NearEqual(jfac*w[0], exp_u, eps99 , eps99); // d^jz/du^j
        v[0]  = 0.;
    }

    // reverse computation of partials of Taylor coefficients
    CPPAD_TESTVECTOR(double) r(p);
    w[0]  = 1.;
    r     = f.Reverse(p, w);
    jfac  = 1.;
    for(j = 0; j < p; j++)
    {   ok &= NearEqual(jfac*r[j], exp_u, eps99 , eps99); // d^jz/du^j
        jfac *= double(j + 1);
    }

    return ok;
}

} // END empty namespace

bool Exp(void)
{   bool ok = true;
    ok &= ExpTestOne();
    ok &= ExpTestTwo();
    return ok;
}
