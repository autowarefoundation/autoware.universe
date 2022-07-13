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
Old example now just used for validation testing.
*/

# include <cppad/cppad.hpp>

bool PowInt(void)
{   bool ok = true;
    using CppAD::pow;
    using CppAD::exp;
    using CppAD::log;
    using namespace CppAD;
    double eps99 = 99.0 * std::numeric_limits<double>::epsilon();


    // independent variable vector, indices, values, and declaration
    CPPAD_TESTVECTOR(AD<double>) U(1);
    U[0]     = 2.;
    Independent(U);

    // dependent variable vector and indices
    CPPAD_TESTVECTOR(AD<double>) Z(2);

    // dependent variable values
    Z[0]         = pow(U[0], 5);     // x = u^5
    Z[1]         = pow(U[0], -5);    // y = u^{-5}

    // create f: U -> Z and vectors used for derivative calculations
    ADFun<double> f(U, Z);
    CPPAD_TESTVECTOR(double) v( f.Domain() );
    CPPAD_TESTVECTOR(double) w( f.Range() );

    /*
    x_u = 5 * u^4
    y_u = - 5 * u^{-6}
    */

    // check function values values
    double u = Value(U[0]);
    ok &= NearEqual(Z[0] , exp( log(u) * 5.),              eps99 , eps99);
    ok &= NearEqual(Z[1] , exp( - log(u) * 5.),            eps99 , eps99);

    // forward computation of partials
    v[0] = 1.;
    w = f.Forward(1, v);
    ok &= NearEqual(w[0] , 5. * exp( log(u) * 4.),         eps99 , eps99);
    ok &= NearEqual(w[1] , - 5. * exp( - log(u) * 6.),     eps99 , eps99);

    return ok;
}
