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
Old DivEq example now used just for valiadation testing
*/
# include <cppad/cppad.hpp>

namespace { // BEGIN empty namespace

bool DivEqTestOne(void)
{   bool ok = true;

    using namespace CppAD;
    using CppAD::NearEqual;
    double eps99 = 99.0 * std::numeric_limits<double>::epsilon();

    // independent variable vector, indices, values, and declaration
    CPPAD_TESTVECTOR(AD<double>) U(2);
    size_t s = 0;
    size_t t = 1;
    U[s]     = 3.;
    U[t]     = 2.;
    Independent(U);

    // dependent variable vector and indices
    CPPAD_TESTVECTOR(AD<double>) Z(2);
    size_t x = 0;
    size_t y = 1;

    // some constants
    AD<double> zero = 0.;
    AD<double> one  = 1.;

    // dependent variable values
    Z[x] = U[s];
    Z[y] = U[t];
    Z[x] /= U[t]; // AD<double> *= AD<double>
    Z[y] /= 5.;   // AD<double> *= double
    zero /= Z[y]; // divide into a parameter equal to zero
    Z[y] /= one;  // divide by a parameter equal to one
    Z[y] /= 1.;   // divide by a double equal to one

    // check that zero is still a parameter
    // (must do this before creating f because it erases the tape)
    ok &= Parameter(zero);

    // create f : U -> Z and vectors for derivative calcualtions
    ADFun<double> f(U, Z);
    CPPAD_TESTVECTOR(double) v( f.Domain() );
    CPPAD_TESTVECTOR(double) w( f.Range() );

    // check that none of the components of f are parameters
    size_t i;
    for(i = 0; i < f.Range(); i++)
        ok &= ! f.Parameter(i);

    // check functin values
    ok &= NearEqual(Z[x] , 3. / 2. , eps99, eps99);
    ok &= NearEqual(Z[y] , 2. / 5. , eps99, eps99);

    // forward computation of partials w.r.t. t
    v[s] = 0.;
    v[t] = 1.;
    w    = f.Forward(1, v);
    ok &= NearEqual(w[x] , -1.*U[s]/(U[t]*U[t]) , eps99, eps99); // dx/dt
    ok &= NearEqual(w[y] , 1. / 5.              , eps99, eps99); // dy/dt

    // reverse computation of second partials of x
    CPPAD_TESTVECTOR(double) r( f.Domain() * 2 );
    w[x] = 1.;
    w[y] = 0.;
    r    = f.Reverse(2, w);
    ok &= NearEqual(r[2 * s + 1]                 // d^2 x / (dt ds)
         , - 1. / (U[t] * U[t])     , eps99 , eps99 );
    ok &= NearEqual(r[2 * t + 1]                 // d^2 x / (dt dt)
        , 2. * U[s] / (U[t] * U[t] * U[t])  , eps99 , eps99 );

    return ok;
}

bool DivEqTestTwo(void)
{   bool ok = true;

    using namespace CppAD;
    using CppAD::NearEqual;
    double eps99 = 99.0 * std::numeric_limits<double>::epsilon();

    // independent variable vector
    double u0 = .5;
    CPPAD_TESTVECTOR(AD<double>) U(1);
    U[0]      = u0;
    Independent(U);

    // dependent variable vector
    CPPAD_TESTVECTOR(AD<double>) Z(1);
    Z[0] = U[0] * U[0]; // initial value
    Z[0] /= 2;          // AD<double> /= int
    Z[0] /= 4.;         // AD<double> /= double
    Z[0] /= U[0];       // AD<double> /= AD<double>

    // create f: U -> Z and vectors used for derivative calculations
    ADFun<double> f(U, Z);
    CPPAD_TESTVECTOR(double) v(1);
    CPPAD_TESTVECTOR(double) w(1);

    // check value
    ok &= NearEqual(Z[0] , u0*u0/(2*4*u0), eps99, eps99);

    // forward computation of partials w.r.t. u
    size_t j;
    size_t p     = 5;
    double jfac  = 1.;
    double value = 1./8.;
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
    value = 1./8.;
    for(j = 0; j < p; j++)
    {   ok &= NearEqual(r[j], value/jfac, eps99, eps99); // d^jz/du^j
        jfac *= double(j + 1);
        value = 0.;
    }

    return ok;
}

} // END empty namespace

bool DivEq(void)
{   bool ok = true;
    ok &= DivEqTestOne();
    ok &= DivEqTestTwo();
    return ok;
}
