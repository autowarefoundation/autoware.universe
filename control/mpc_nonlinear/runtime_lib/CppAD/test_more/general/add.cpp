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
Two old Add examples now used just for valiadation testing
*/

# include <cppad/cppad.hpp>

namespace { // BEGIN empty namespace

bool AddTestOne(void)
{   bool ok = true;

    using namespace CppAD;

    // independent variable vector, indices, values, and declaration
    CPPAD_TESTVECTOR(AD<double>) U(2);
    size_t s = 0;
    size_t t = 1;
    U[s]     = 3.;
    U[t]     = 2.;
    Independent(U);

    // dependent variable vector and indices
    CPPAD_TESTVECTOR(AD<double>) Z(3);
    size_t x = 0;
    size_t y = 1;
    size_t z = 2;

    // dependent variable values
    Z[x] = U[s]  + U[t];   // AD<double> + AD<double>
    Z[y] = Z[x]  + 1.;     // AD<double> + double
    Z[z] = 1.    + Z[y];   // double + AD<double>

    // create f: U -> Z and vectors used for derivative calculations
    ADFun<double> f(U, Z);
    CPPAD_TESTVECTOR(double) v( f.Domain() );
    CPPAD_TESTVECTOR(double) w( f.Range() );

    // check function values
    ok &= ( Z[x] == 3. + 2. );
    ok &= ( Z[y] == 3. + 2. + 1. );
    ok &= ( Z[z] == 1. + 3. + 2. + 1. );

    // forward computation of partials w.r.t. s
    v[s] = 1.;
    v[t] = 0.;
    w    = f.Forward(1, v);
    ok &= ( w[x] == 1. );   // dx/ds
    ok &= ( w[y] == 1. );   // dy/ds
    ok &= ( w[z] == 1. );   // dz/ds

    // reverse computation of second partials of z
    CPPAD_TESTVECTOR(double) r( f.Domain() * 2 );
    w[x] = 0.;
    w[y] = 0.;
    w[z] = 1.;
    r    = f.Reverse(2, w);
    ok &= ( r[2 * s + 1] == 0. );  // d^2 z / (ds ds)
    ok &= ( r[2 * t + 1] == 0. );  // d^2 z / (ds dt)

    return ok;
}

bool AddTestTwo(void)
{   bool ok = true;
    using namespace CppAD;
    double eps99 = 99.0 * std::numeric_limits<double>::epsilon();

    // independent variable vector
    double u0 = .5;
    CPPAD_TESTVECTOR(AD<double>) U(1);
    U[0]      = u0;
    Independent(U);

    AD<double> a = U[0] + 1.; // AD<double> + double
    AD<double> b = a  + 2;    // AD<double> + int
    AD<double> c = 3. + b;    // double     + AD<double>
    AD<double> d = 4  + c;    // int        + AD<double>

    // dependent variable vector
    CPPAD_TESTVECTOR(AD<double>) Z(1);
    Z[0] = d + U[0];          // AD<double> + AD<double>

    // create f: U -> Z and vectors used for derivative calculations
    ADFun<double> f(U, Z);
    CPPAD_TESTVECTOR(double) v(1);
    CPPAD_TESTVECTOR(double) w(1);

    // check value
    ok &= NearEqual(Z[0] , 2 * u0 + 10,  eps99 , eps99);

    // forward computation of partials w.r.t. u
    size_t j;
    size_t p     = 5;
    double jfac  = 1.;
    double value = 2.;
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
    value = 2.;
    for(j = 0; j < p; j++)
    {   ok &= NearEqual(r[j], value/jfac, eps99, eps99); // d^jz/du^j
        jfac *= double(j + 1);
        value = 0.;
    }

    return ok;
}

} // END empty namespace

bool Add(void)
{   bool ok = true;
    ok &= AddTestOne();
    ok &= AddTestTwo();
    return ok;
}
