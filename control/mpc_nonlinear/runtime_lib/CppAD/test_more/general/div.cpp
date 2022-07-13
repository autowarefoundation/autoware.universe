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
Two old Div examples now used just for valiadation testing
*/

# include <cppad/cppad.hpp>

namespace { // BEGIN empty namespace

bool DivTestOne(void)
{   bool ok = true;

    using namespace CppAD;
    using CppAD::NearEqual;
    double eps99 = 99.0 * std::numeric_limits<double>::epsilon();

    // assign some parameters
    AD<double> zero = 0.;
    AD<double>  one = 1.;

    // independent variable vector, indices, values, and declaration
    CPPAD_TESTVECTOR(AD<double>) U(2);
    size_t s = 0;
    size_t t = 1;
    U[s]     = 2.;
    U[t]     = 3.;
    Independent(U);

    // dependent variable vector and indices
    CPPAD_TESTVECTOR(AD<double>) Z(6);
    size_t x = 0;
    size_t y = 1;
    size_t z = 2;
    size_t u = 3;
    size_t v = 4;
    size_t w = 5;

    // dependent variables
    Z[x] = U[s]   / U[t];   // AD<double> / AD<double>
    Z[y] = Z[x]   / 4.;     // AD<double> / double
    Z[z] = 5. / Z[y];       //     double / AD<double>
    Z[u] =  Z[z] / one;     // division by a parameter equal to one
    Z[v] =  Z[z] / 1.;      // division by a double equal to one
    Z[w] =  zero / Z[z];    // division into a parameter equal to zero

    // check division into a zero valued parameter results in a parameter
    // (must do this before creating f because it erases the tape)
    ok &= Parameter(Z[w]);

    // create f : U -> Z and vectors used for derivative calculations
    ADFun<double> f(U, Z);
    CPPAD_TESTVECTOR(double) q( f.Domain() );
    CPPAD_TESTVECTOR(double) r( f.Range() );

    // check parameter flag
    ok &= f.Parameter(w);

    // check values
    ok &= NearEqual( Z[x] , 2. / 3. , eps99, eps99);
    ok &= NearEqual( Z[y] , 2. / ( 3. * 4. ) , eps99, eps99);
    ok &= NearEqual( Z[z] , 5. * 3. * 4. / 2. , eps99, eps99);
    ok &= ( Z[w] == 0. );
    ok &= ( Z[u] == Z[z] );

    // forward computation of partials w.r.t. s
    q[s] = 1.;
    q[t] = 0.;
    r    = f.Forward(1, q);
    ok &= NearEqual(r[x], 1./U[t], eps99, eps99); // dx/ds
    ok &= NearEqual(r[y], 1./(U[t]*4.), eps99, eps99); // dy/ds
    ok &= NearEqual(r[z], -5.*U[t]*4./(U[s]*U[s]), eps99, eps99); // dz/ds
    ok &= ( r[u] == r[z] );                                       // du/ds
    ok &= ( r[v] == r[z] );                                       // dv/ds
    ok &= ( r[w] == 0. );                                         // dw/ds

    // forward computation in the direction (1, 1)
    q[s] = 1.;
    q[t] = 1.;
    r    = f.Forward(1, q);
    ok  &= NearEqual(r[x], 1./U[t] - U[s]/(U[t] * U[t]), eps99, eps99);

    // second order reverse mode computation
    CPPAD_TESTVECTOR(double) Q( f.Domain() * 2 );
    r[x] = 1.;
    r[y] = r[z] = r[u] = r[v] = r[w] = 0.;
    Q    = f.Reverse(2, r);
    ok  &= NearEqual(
        Q[s * f.Domain() + 1],
        - 1. / (U[t] * U[t]),
        eps99,
        eps99
    );

    return ok;
}

bool DivTestTwo(void)
{   bool ok = true;
    using namespace CppAD;
    using CppAD::NearEqual;
    double eps99 = 99.0 * std::numeric_limits<double>::epsilon();

    // independent variable vector
    double u0 = .5;
    CPPAD_TESTVECTOR(AD<double>) U(1);
    U[0]      = u0;
    Independent(U);

    AD<double> a = U[0] / 1.; // AD<double> / double
    AD<double> b = a  / 2;    // AD<double> / int
    AD<double> c = 3. / b;    // double     / AD<double>
    AD<double> d = 4  / c;    // int        / AD<double>

    // dependent variable vector
    CPPAD_TESTVECTOR(AD<double>) Z(1);
    Z[0] = U[0] * U[0] / d;   // AD<double> / AD<double>

    // create f: U -> Z and vectors used for derivative calculations
    ADFun<double> f(U, Z);
    CPPAD_TESTVECTOR(double) v(1);
    CPPAD_TESTVECTOR(double) w(1);

    // check value
    ok &= NearEqual(Value(Z[0]) , u0*u0/(4/(3/(u0/2))), eps99, eps99);

    // forward computation of partials w.r.t. u
    size_t j;
    size_t p     = 5;
    double jfac  = 1.;
    v[0]         = 1.;
    double value = 6. / 4.;
    for(j = 1; j < p; j++)
    {
        jfac *= double(j);
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
    value = 6. / 4.;
    for(j = 0; j < p; j++)
    {
        ok &= NearEqual(r[j], value/jfac, eps99, eps99); // d^jz/du^j
        jfac *= double(j + 1);
        value = 0.;
    }

    return ok;
}

bool DivTestThree(void)
{   bool ok = true;
    using namespace CppAD;
    using CppAD::NearEqual;
    double eps99 = 99.0 * std::numeric_limits<double>::epsilon();

    // more testing of variable / variable case
    double x0 = 2.;
    double x1 = 3.;
    size_t n  = 2;
    CPPAD_TESTVECTOR(AD<double>) X(n);
    X[0]      = x0;
    X[1]      = x1;
    Independent(X);
    size_t m  = 1;
    CPPAD_TESTVECTOR(AD<double>) Y(m);
    Y[0]      = X[0] / X[1];
    ADFun<double> f(X, Y);

    CPPAD_TESTVECTOR(double) dx(n), dy(m);
    double check;
    dx[0] = 1.;
    dx[1] = 1.;
    dy    = f.Forward(1, dx);
    check = 1. / x1 - x0 / (x1 * x1);
    ok   &= NearEqual(dy[0], check, eps99, eps99);

    CPPAD_TESTVECTOR(double) w(m), dw(n);
    w[0]  = 1.;
    dw    = f.Reverse(1, w);
    check = 1. / x1;
    ok   &= NearEqual(dw[0], check, eps99, eps99);
    check = - x0 / (x1 * x1);
    ok   &= NearEqual(dw[1], check, eps99, eps99);

    return ok;
}

} // END empty namespace

bool Div(void)
{   bool ok = true;
    ok &= DivTestOne();
    ok &= DivTestTwo();
    ok &= DivTestThree();
    return ok;
}
