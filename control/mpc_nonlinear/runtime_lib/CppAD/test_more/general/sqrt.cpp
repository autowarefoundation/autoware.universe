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
Two old sqrt examples now used just for validation testing.
*/
# include <cppad/cppad.hpp>
# include <cmath>

namespace { // BEGIN empty namespace

bool SqrtTestOne(void)
{   bool ok = true;
    using CppAD::sqrt;
    using CppAD::pow;
    using namespace CppAD;
    double eps99 = 99.0 * std::numeric_limits<double>::epsilon();

    // independent variable vector, indices, values, and declaration
    CPPAD_TESTVECTOR(AD<double>) U(1);
    size_t s = 0;
    U[s]     = 4.;
    Independent(U);

    // dependent variable vector, indices, and values
    CPPAD_TESTVECTOR(AD<double>) Z(2);
    size_t x = 0;
    size_t y = 1;
    Z[x]     = sqrt(U[s]);
    Z[y]     = sqrt(Z[x]);

    // define f : U -> Z and vectors for derivative calculations
    ADFun<double> f(U, Z);
    CPPAD_TESTVECTOR(double) v( f.Domain() );
    CPPAD_TESTVECTOR(double) w( f.Range() );

    // check values
    ok &= NearEqual(Z[x] , 2.,        eps99 , eps99);
    ok &= NearEqual(Z[y] , sqrt(2.),  eps99 , eps99);

    // forward computation of partials w.r.t. s
    v[s] = 1.;
    w    = f.Forward(1, v);
    ok &= NearEqual(w[x], .5  * pow(4., -.5),   eps99 , eps99); // dx/ds
    ok &= NearEqual(w[y], .25 * pow(4., -.75),  eps99 , eps99); // dy/ds

    // reverse computation of partials of y
    w[x] = 0.;
    w[y] = 1.;
    v    = f.Reverse(1,w);
    ok &= NearEqual(v[s], .25 * pow(4., -.75),  eps99 , eps99); // dy/ds

    // forward computation of second partials w.r.t s
    v[s] = 1.;
    w    = f.Forward(1, v);
    v[s] = 0.;
    w    = f.Forward(2, v);
    ok &= NearEqual(       // d^2 y / (ds ds)
        2. * w[y] ,
        -.75 * .25 * pow(4., -1.75),
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
        -.75 * .25 * pow(4., -1.75),
        eps99 ,
        eps99
    );

    return ok;

}
bool SqrtTestTwo(void)
{   bool ok = true;
    using namespace CppAD;
    double eps99 = 99.0 * std::numeric_limits<double>::epsilon();

    // independent variable vector
    CPPAD_TESTVECTOR(AD<double>) U(1);
    U[0]     = 2.;
    Independent(U);

    // a temporary values
    AD<double> x = U[0] * U[0];

    // dependent variable vector
    CPPAD_TESTVECTOR(AD<double>) Z(1);
    Z[0] =  sqrt( x ); // z = sqrt( u * u )

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
bool SqrtTestThree(void)
{   bool ok = true;
    using CppAD::sqrt;
    using CppAD::exp;
    using namespace CppAD;
    double eps99 = 99.0 * std::numeric_limits<double>::epsilon();

    // independent variable vector, indices, values, and declaration
    double x = 4.;
    CPPAD_TESTVECTOR(AD<double>) X(1);
    X[0]     = x;
    Independent(X);

    // dependent variable vector, indices, and values
    CPPAD_TESTVECTOR(AD<double>) Y(1);
    Y[0]     = sqrt( exp(X[0]) );

    // define f : X -> Y and vectors for derivative calculations
    ADFun<double> f(X, Y);

    // forward computation of first Taylor coefficient
    CPPAD_TESTVECTOR(double) x1( f.Domain() );
    CPPAD_TESTVECTOR(double) y1( f.Range() );
    x1[0] = 1.;
    y1    = f.Forward(1, x1);
    ok   &= NearEqual(y1[0], exp(x/2.)/2.,   eps99 , eps99);

    // forward computation of second Taylor coefficient
    CPPAD_TESTVECTOR(double) x2( f.Domain() );
    CPPAD_TESTVECTOR(double) y2( f.Range() );
    x2[0] = 0.;
    y2    = f.Forward(2, x2);
    ok   &= NearEqual(2.*y2[0] , exp(x/2.)/4., eps99 , eps99 );

    // forward computation of third Taylor coefficient
    CPPAD_TESTVECTOR(double) x3( f.Domain() );
    CPPAD_TESTVECTOR(double) y3( f.Range() );
    x3[0] = 0.;
    y3    = f.Forward(3, x3);
    ok   &= NearEqual(6.*y3[0] , exp(x/2.)/8., eps99 , eps99 );

    // reverse computation of deritavitve of Taylor coefficients
    CPPAD_TESTVECTOR(double) r( f.Domain() * 4 );
    CPPAD_TESTVECTOR(double) w(1);
    w[0] = 1.;
    r    = f.Reverse(4, w);
    ok   &= NearEqual(r[0], exp(x/2.)/2., eps99 , eps99);
    ok   &= NearEqual(r[1], exp(x/2.)/4., eps99 , eps99 );
    ok   &= NearEqual(2.*r[2], exp(x/2.)/8., eps99 , eps99 );
    ok   &= NearEqual(6.*r[3], exp(x/2.)/16., eps99 , eps99 );

    return ok;

}

} // END empty namespace

bool Sqrt(void)
{   bool ok = true;
    ok &= SqrtTestOne();
    ok &= SqrtTestTwo();
    ok &= SqrtTestThree();
    return ok;
}
