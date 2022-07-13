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
Two old Mul examples now used just for valiadation testing
*/
# include <cppad/cppad.hpp>

namespace { // BEGIN empty namespace

bool MulTestOne(void)
{   bool ok = true;

    using namespace CppAD;

    // independent variable vector, indices, values, and declaration
    CPPAD_TESTVECTOR(AD<double>) U(2);
    size_t s = 0;
    size_t t = 1;
    U[s]     = 3.;
    U[t]     = 2.;
    Independent(U);

    // assign some parameters
    AD<double> zero = 0.;
    AD<double> one  = 1.;

    // dependent variable vector and indices
    CPPAD_TESTVECTOR(AD<double>) Z(5);
    size_t x = 0;
    size_t y = 1;
    size_t z = 2;
    size_t u = 3;
    size_t v = 4;

    // assign the dependent variables
    Z[x] = U[s] * U[t];   // AD<double> * AD<double>
    Z[y] = Z[x] * 4.;     // AD<double> *    double
    Z[z] = 4.   * Z[y];   //    double  * AD<double>
    Z[u] =  one * Z[z];   // multiplication by parameter equal to one
    Z[v] = zero * Z[z];   // multiplication by parameter equal to zero

    // check multipilcation by zero results in a parameter
    ok &= Parameter(Z[v]);

    // create f: U -> Z and vectors used for derivative calculations
    ADFun<double> f(U, Z);
    CPPAD_TESTVECTOR(double) q( f.Domain() );
    CPPAD_TESTVECTOR(double) r( f.Range() );

    // check parameter flag
    ok &= f.Parameter(v);

    // check values
    ok &= ( Z[x] == 3. * 2. );
    ok &= ( Z[y] == 3. * 2. * 4. );
    ok &= ( Z[z] == 4. * 3. * 2. * 4. );
    ok &= ( Z[u] == Z[z] );
    ok &= ( Z[v] == 0. );

    // forward computation of partials w.r.t. s
    q[s] = 1.;
    q[t] = 0.;
    r    = f.Forward(1, q);
    ok &= ( r[x] == U[t] );           // dx/ds
    ok &= ( r[y] == U[t] * 4. );      // dy/ds
    ok &= ( r[z] == 4. * U[t] * 4. ); // dz/ds
    ok &= ( r[u] == r[z] );           // du/ds
    ok &= ( r[v] == 0. );             // dv/ds

    // reverse computation of second partials of z
    CPPAD_TESTVECTOR(double) d2( f.Domain() * 2 );
    r[x] = 0.;
    r[y] = 0.;
    r[z] = 1.;
    r[u] = 0.;
    r[v] = 0.;
    d2   = f.Reverse(2, r);

    // check second order partials
    ok &= ( d2[2 * s + 1] == 0. );             // d^2 z / (ds ds)
    ok &= ( d2[2 * t + 1] == 4. * 4. );        // d^2 z / (ds dt)

    return ok;
}

bool MulTestTwo(void)
{   bool ok = true;
    using namespace CppAD;
    double eps99 = 99.0 * std::numeric_limits<double>::epsilon();

    // independent variable vector
    double u0 = .5;
    CPPAD_TESTVECTOR(AD<double>) U(1);
    U[0]      = u0;
    Independent(U);

    AD<double> a = U[0] * 1.; // AD<double> * double
    AD<double> b = a  * 2;    // AD<double> * int
    AD<double> c = 3. * b;    // double     * AD<double>
    AD<double> d = 4  * c;    // int        * AD<double>

    // dependent variable vector
    CPPAD_TESTVECTOR(AD<double>) Z(1);
    Z[0] = U[0] * d;          // AD<double> * AD<double>

    // create f: U -> Z and vectors used for derivative calculations
    ADFun<double> f(U, Z);
    CPPAD_TESTVECTOR(double) v(1);
    CPPAD_TESTVECTOR(double) w(1);

    // check value
    ok &= NearEqual(Value(Z[0]) , u0*4*3*2*u0,  eps99 , eps99);

    // forward computation of partials w.r.t. u
    size_t j;
    size_t p     = 5;
    double jfac  = 1.;
    v[0]         = 1.;
    for(j = 1; j < p; j++)
    {   double value;
        if( j == 1 )
            value = 48. * u0;
        else if( j == 2 )
            value = 48.;
        else
            value = 0.;

        jfac *= double(j);
        w     = f.Forward(j, v);
        ok &= NearEqual(w[0], value/jfac, eps99, eps99); // d^jz/du^j
        v[0]  = 0.;
    }

    // reverse computation of partials of Taylor coefficients
    CPPAD_TESTVECTOR(double) r(p);
    w[0]  = 1.;
    r     = f.Reverse(p, w);
    jfac  = 1.;
    for(j = 0; j < p; j++)
    {   double value;
        if( j == 0 )
            value = 48. * u0;
        else if( j == 1 )
            value = 48.;
        else
            value = 0.;

        ok &= NearEqual(r[j], value/jfac, eps99, eps99); // d^jz/du^j
        jfac *= double(j + 1);
    }

    return ok;
}

} // END empty namespace

bool Mul(void)
{   bool ok = true;
    ok &= MulTestOne();
    ok &= MulTestTwo();
    return ok;
}
