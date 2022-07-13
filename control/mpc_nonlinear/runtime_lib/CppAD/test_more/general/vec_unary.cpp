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
Test the use of VecADelem with unary operators
*/

# include <cppad/cppad.hpp>


bool VecUnary(void)
{
    using namespace CppAD;
    using CppAD::sin;
    using CppAD::atan;
    using CppAD::cos;
    using CppAD::exp;
    using CppAD::log;
    using CppAD::sqrt;
    using CppAD::NearEqual;
    double eps99 = 99.0 * std::numeric_limits<double>::epsilon();

    bool ok  = true;
    size_t n = 8;
    size_t i;

    CPPAD_TESTVECTOR(AD<double>) X(n);
    VecAD<double>             Y(n);
    CPPAD_TESTVECTOR(AD<double>) Z(n);


    for(i = 0; i < n; i++)
        X[i] = int(i);  // some compilers require the int here
    Independent(X);

    AD<double> j;

    j    = 0.;
    Y[j] = X[0];
    Z[0] = -Y[j];

    j    = 1.;
    Y[j] = X[1];
    Z[1] = sin( Y[j] );

    j    = 2.;
    Y[j] = X[2];
    Z[2] = fabs( Y[j] );

    j    = 3.;
    Y[j] = X[3];
    Z[3] = atan( Y[j] );

    j    = 4.;
    Y[j] = X[4];
    Z[4] = cos( Y[j] );

    j    = 5.;
    Y[j] = X[5];
    Z[5] = exp( Y[j] );

    j    = 6.;
    Y[j] = X[6];
    Z[6] = log( Y[j] );

    j    = 7.;
    Y[j] = X[7];
    Z[7] = sqrt( Y[j] );


    ADFun<double> f(X, Z);
    CPPAD_TESTVECTOR(double) x(n);
    CPPAD_TESTVECTOR(double) z(n);

    for(i = 0; i < n; i++)
        x[i] = 2. / double(i + 1);
    x[7] = fabs( x[7] );

    z    = f.Forward(0, x);

    ok  &= NearEqual(z[0],      - x[0], eps99, eps99);
    ok  &= NearEqual(z[1], sin( x[1] ), eps99, eps99);
    ok  &= NearEqual(z[2], fabs( x[2] ), eps99, eps99);
    ok  &= NearEqual(z[3], atan(x[3] ), eps99, eps99);
    ok  &= NearEqual(z[4], cos( x[4] ), eps99, eps99);
    ok  &= NearEqual(z[5], exp( x[5] ), eps99, eps99);
    ok  &= NearEqual(z[6], log( x[6] ), eps99, eps99);
    ok  &= NearEqual(z[7], sqrt(x[7] ), eps99, eps99);

    return ok;
}
