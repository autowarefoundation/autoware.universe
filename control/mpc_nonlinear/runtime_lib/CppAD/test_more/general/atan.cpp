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
Two old atan examples now used just for validation testing.
*/

# include <cppad/cppad.hpp>

namespace { // BEGIN empty namespace

bool AtanTestOne(void)
{   bool ok = true;

    using CppAD::atan;
    using namespace CppAD;
    double eps99 = 99.0 * std::numeric_limits<double>::epsilon();

    // independent variable vector, indices, values, and declaration
    CPPAD_TESTVECTOR(AD<double>) U(1);
    size_t s = 0;
    U[s]     = 1.;
    Independent(U);

    // some temporary values
    AD<double> x = cos(U[s]);
    AD<double> y = sin(U[s]);
    AD<double> z = y / x;       // tan(s)

    // dependent variable vector and indices
    CPPAD_TESTVECTOR(AD<double>) Z(1);
    size_t a = 0;

    // dependent variable values
    Z[a] = atan(z); // atan( tan(s) )

    // create f: U -> Z and vectors used for dierivative calculations
    ADFun<double> f(U, Z);
    CPPAD_TESTVECTOR(double) v( f.Domain() );
    CPPAD_TESTVECTOR(double) w( f.Range() );

    // check value
    ok &= NearEqual(U[s] , Z[a],  eps99 , eps99);

    // forward computation of partials w.r.t. s
    v[s] = 1.;
    w    = f.Forward(1, v);
    ok &= NearEqual(w[a], 1e0, eps99 , eps99);  // da/ds

    // reverse computation of first order partial of a
    w[a] = 1.;
    v    = f.Reverse(1, w);
    ok &= NearEqual(v[s], 1e0, eps99 , eps99);  // da/ds

    // forward computation of second partials w.r.t. s and s
    v[s] = 1.;
    f.Forward(1, v);
    v[s] = 0.;
    w    = f.Forward(2, v);
    ok &= NearEqual(2. * w[a], 0e0, eps99 , eps99);     // d^2 a / (ds ds)

    // reverse computation of second partials of a
    CPPAD_TESTVECTOR(double) r( f.Domain() * 2 );
    w[a] = 1.;
    r    = f.Reverse(2, w);
    ok &= NearEqual(r[2 * s + 1] ,0e0, eps99 , eps99 ); // d^2 a / (ds ds)

    return ok;
}

bool AtanTestTwo(void)
{   bool ok = true;

    using CppAD::atan;
    using CppAD::sin;
    using CppAD::cos;
    using namespace CppAD;
    double eps99 = 99.0 * std::numeric_limits<double>::epsilon();

    // independent variable vector
    CPPAD_TESTVECTOR(AD<double>) U(1);
    U[0]     = 1.;
    Independent(U);

    // a temporary values
    AD<double> x = sin(U[0]) / cos(U[0]);

    // dependent variable vector
    CPPAD_TESTVECTOR(AD<double>) Z(1);
    Z[0] = atan(x); // atan( tan(u) )

    // create f: U -> Z and vectors used for derivative calculations
    ADFun<double> f(U, Z);
    CPPAD_TESTVECTOR(double) v(1);
    CPPAD_TESTVECTOR(double) w(1);

    // check value
    ok &= NearEqual(U[0] , Z[0],  eps99 , eps99);

    // forward computation of partials w.r.t. u
    size_t j;
    size_t p     = 5;
    double jfac  = 1.;
    double value = 1.;
    v[0]         = 1.;
    for(j = 1; j < p; j++)
    {   jfac *= double(j);
        w     = f.Forward(j, v);
        ok &= NearEqual(w[0], value/jfac, eps99, eps99);// d^jz/du^j
        v[0]  = 0.;
        value = 0.;
    }

    // reverse computation of partials of Taylor coefficients
    CPPAD_TESTVECTOR(double) r(p);
    w[0]  = 1.;
    r     = f.Reverse(p, w);
    jfac  = 1.;
    value = 1.;
    for(j = 0; j < p; j++)
    {   ok &= NearEqual(r[j], value/jfac, eps99, eps99);// d^jz/du^j
        jfac *= double(j + 1);
        value = 0.;
    }

    return ok;
}

} // END empty namespace

bool atan(void)
{   bool ok = true;
    ok &= AtanTestOne();
    ok &= AtanTestTwo();
    return ok;
}
