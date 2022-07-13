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
*/
// BEGIN C++
# include <cppad/cppad.hpp>

namespace {

    double my_discrete(const double& x)
    {   return static_cast<int> ( x ); }
    CPPAD_DISCRETE_FUNCTION(double, my_discrete)

}
bool forward_order(void)
{   bool ok = true;
    using CppAD::AD;
    using CppAD::NearEqual;
    size_t j, k;
    double eps = 10. * CppAD::numeric_limits<double>::epsilon();

    // domain space vector
    size_t n = 23, m = n;
    CPPAD_TESTVECTOR(AD<double>) X(n), Y(m);
    for(j = 0; j < n; j++)
        X[j] = 0.0;

    // declare independent variables and starting recording
    CppAD::Independent(X);

    // identity function values
    size_t i = 0;
    size_t identity_begin = i;
    Y[i] = cos( acos( X[i] ) );                   i++; // AcosOp,  CosOp
    Y[i] = sin( asin( X[i] ) );                   i++; // AsinOp,  SinOp
    Y[i] = tan( atan( X[i] ) );                   i++; // AtanOp,  TanOp
    Y[i] = CondExpGt(X[i], X[i-1], X[i], X[i-2]); i++; // CExpOp
    Y[i] = X[i-1] * X[i] / X[i-1];                i++; // DivvvOp, MulvvOp
    Y[i] = X[i] * X[i] * 1.0 / X[i];              i++; // DivpvOp
    Y[i] = 5.0 * X[i] / 5.0;                      i++; // DivvpOp, MulpvOp
    Y[i] = exp( log( X[i] ) );                    i++; // ExpOp,   LogOp
    Y[i] = pow( sqrt( X[i] ), 2.0);               i++; // PowvpOp, SqrtOp
    Y[i] = log( pow( std::exp(1.), X[i] ) );      i++; // PowpvOp
    Y[i] = log( pow( X[i], X[i] ) ) / log( X[i]); i++; // PowvvOp
    Y[i] = -2. - ((X[i-1] - X[i]) - 2.) + X[i-1]; i++; // Sub*Op: pv, vv, vp
    size_t identity_end = i;

    // other functions
    Y[i] = fabs( X[i] );        i++;   // AbsOp
    Y[i] = X[i-1] + X[i] + 2.0; i++;   // AddvvOp, AddvpOp
    Y[i] = cosh( X[i] );        i++;   // CoshOp
    Y[i] = my_discrete( X[i] ); i++;   // DisOp
    Y[i] = 4.0;                 i++;   // ParOp
    Y[i] = sign( X[i] );        i++;   // SignOp
    Y[i] = sinh( X[i] );        i++;   // SinhOp
    Y[i] = tanh(X[i]);          i++;   // TanhOp

    // VecAD operations
    CppAD::VecAD<double> V(n);
    AD<double> index = 1.;
    V[index] = 3.0;
    Y[i]     = V[index];            i++;   // StppOp, LdpOp
    V[index] = X[0];
    Y[i]     = V[index];            i++;   // StpvOp, LdpOp
    index    = double(n) * X[3];
    V[index] = X[1];
    Y[i]     = V[index];            i++;   // StvvOp, LdvOp

    // create f: X -> Y and stop tape recording
    assert( i == m );
    CppAD::ADFun<double> f;
    f.Dependent(X, Y);

    // initially, no values stored in f
    ok &= f.size_order() == 0;

    // Set X_j (t) = x + t
    size_t p = 2, p1 = p+1;
    CPPAD_TESTVECTOR(double) x(n), x_p(n * p1), y_p(m * p1);
    for(j = 0; j < n; j++)
    {   x[j]            = double(j) / double(n);
        x_p[j * p1 + 0] = x[j]; // order 0
        x_p[j * p1 + 1] = 1.;   // order 1
        x_p[j * p1 + 2] = 0.;   // order 2
    }
    // compute orders 0, 1, 2
    y_p  = f.Forward(p, x_p);

    // identity functions
    CPPAD_TESTVECTOR(double) y(p1);
    i = 0;
    for(j = identity_begin; j != identity_end; j++)
    {   y[0] = x[j];
        y[1] = 1.0;
        y[2] = 0.0;
        for(k = 0; k < p1; k++)
            ok  &= NearEqual(y[k] , y_p[i * p1 + k], eps, eps);
        i++;
    }

    // y_i = fabs( x_i )
    y[0] = fabs( x[i] );
    y[1] = CppAD::sign( x[i] );
    y[2] = 0.0;
    for(k = 0; k < p1; k++)
        ok  &= NearEqual(y[k] , y_p[i * p1 + k], eps, eps);

    // y_i = x_[i-1] + x_i + 2
    i++;
    y[0] = x[i-1] + x[i] + 2.0;
    y[1] = 2.0;
    y[2] = 0.0;
    for(k = 0; k < p1; k++)
        ok  &= NearEqual(y[k] , y_p[i * p1 + k], eps, eps);

    // y_i = cosh( x_i )
    i++;
    y[0] = CppAD::cosh( x[i] );
    y[1] = CppAD::sinh( x[i] );
    y[2] = CppAD::cosh( x[i] ) / 2.0;
    for(k = 0; k < p1; k++)
        ok  &= NearEqual(y[k] , y_p[i * p1 + k], eps, eps);

    // y_i = my_discrete( x_i )
    i++;
    y[0] = my_discrete( x[i] );
    y[1] = 0.0;
    y[2] = 0.0;
    for(k = 0; k < p1; k++)
        ok  &= NearEqual(y[k] , y_p[i * p1 + k], eps, eps);

    // y_i = 4
    i++;
    y[0] = 4.0;
    y[1] = 0.0;
    y[2] = 0.0;
    for(k = 0; k < p1; k++)
        ok  &= NearEqual(y[k] , y_p[i * p1 + k], eps, eps);

    // y_i = sign( x_i )
    i++;
    y[0] = CppAD::sign( x[i] );
    y[1] = 0.0;
    y[2] = 0.0;
    for(k = 0; k < p1; k++)
        ok  &= NearEqual(y[k] , y_p[i * p1 + k], eps, eps);

    // y_i = sinh( x_i )
    i++;
    y[0] = CppAD::sinh( x[i] );
    y[1] = CppAD::cosh( x[i] );
    y[2] = CppAD::sinh( x[i] ) / 2.0;
    for(k = 0; k < p1; k++)
        ok  &= NearEqual(y[k] , y_p[i * p1 + k], eps, eps);

    // y_i = tanh( x_i )
    i++;
    y[0] = CppAD::tanh( x[i] );
    y[1] = 1.0 - y[0] * y[0];
    y[2] = - 2.0 * y[0] * y[1] / 2.0;
    for(k = 0; k < p1; k++)
        ok  &= NearEqual(y[k] , y_p[i * p1 + k], eps, eps);

    // y_i = 3.0;
    i++;
    y[0] = 3.0;
    y[1] = 0.0;
    y[2] = 0.0;
    for(k = 0; k < p1; k++)
        ok  &= NearEqual(y[k] , y_p[i * p1 + k], eps, eps);

    // y_i = x_0
    i++;
    y[0] = x[0];
    y[1] = 1.0;
    y[2] = 0.0;
    for(k = 0; k < p1; k++)
        ok  &= NearEqual(y[k] , y_p[i * p1 + k], eps, eps);

    // y_i = x_1
    i++;
    y[0] = x[1];
    y[1] = 1.0;
    y[2] = 0.0;
    for(k = 0; k < p1; k++)
        ok  &= NearEqual(y[k] , y_p[i * p1 + k], eps, eps);

    return ok;
}
// END C++
