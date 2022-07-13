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
Old example and test now just used for validation testing.
*/

# include <cppad/cppad.hpp>
# include <cmath>

namespace { // begin empty namespace

bool ad_ad(void)
{   bool ok = true;
    using CppAD::AD;
    using CppAD::NearEqual;
    double eps99 = 99.0 * std::numeric_limits<double>::epsilon();

    AD< AD<double> > x(2.), y(2.);
    AD< AD<double> > z = atan2(y, x);
    NearEqual( Value( Value(z) ), atan(1.), eps99, eps99);

    return ok;
}

bool general(void)
{   bool ok = true;

    using CppAD::atan;
    using CppAD::sin;
    using CppAD::cos;
    using namespace CppAD;
    using CppAD::NearEqual;
    double eps99 = 99.0 * std::numeric_limits<double>::epsilon();

    // independent variable vector
    CPPAD_TESTVECTOR(AD<double>) U(1);
    U[0]     = 1.;
    Independent(U);

    // a temporary values
    AD<double> x = cos(U[0]);
    AD<double> y = sin(U[0]);

    // dependent variable vector
    CPPAD_TESTVECTOR(AD<double>) Z(1);
    Z[0] = atan2(y, x);

    // create f: U -> Z and vectors used for derivative calculations
    ADFun<double> f(U, Z);
    CPPAD_TESTVECTOR(double) v(1);
    CPPAD_TESTVECTOR(double) w(1);

    // check original value (u in first quadrant)
    ok &= NearEqual(U[0] , Z[0], eps99, eps99);

    // check case where u is in second quadrant
    v[0] = 3.;
    w    = f.Forward(0, v);
    ok  &= NearEqual(w[0] , v[0], eps99, eps99);

    // check case where u is in third quadrant
    v[0] = -3.;
    w    = f.Forward(0, v);
    ok  &= NearEqual(w[0] , v[0], eps99, eps99);

    // check case where u is in fourth quadrant
    v[0] = -1.;
    w    = f.Forward(0, v);
    ok  &= NearEqual(w[0] , v[0], eps99, eps99);

    // forward computation of partials w.r.t. u
    size_t j;
    size_t p     = 5;
    double jfac  = 1.;
    double value = 1.;
    v[0]         = 1.;
    for(j = 1; j < p; j++)
    {   jfac *= double(j);
        w     = f.Forward(j, v);
        ok &= NearEqual(w[0], value/jfac, eps99, eps99); // d^jz/du^j
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
    {   ok &= NearEqual(r[j], value/jfac, eps99, eps99); // d^jz/du^j
        jfac *= double(j + 1);
        value = 0.;
    }

    return ok;
}

} // end empty namespace

bool atan2(void)
{   bool ok = true;
    ok     &= ad_ad();
    ok     &= general();

    return ok;
}
