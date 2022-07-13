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
Old Copy example now used just for valiadation testing
*/

# include <cppad/cppad.hpp>

namespace { // begin empty namespace

bool copy_older(void)
{   bool ok = true;

    using namespace CppAD;

    // independent variable vector, indices, values, and declaration
    CPPAD_TESTVECTOR(AD<double>) U(1);
    size_t is = 0;
    U[is]     = 2.;
    Independent(U);

    // create an AD<double> that does not depend on s
    AD<double> t = 3.;

    // use copy constructor
    AD<double> x(U[is]);
    AD<double> y(t);

    // check which are parameters
    ok &= ! Parameter(x);
    ok &= Parameter(y);

    // dependent variable vector, indices, and values
    CPPAD_TESTVECTOR(AD<double>) Z(2);
    size_t ix = 0;
    size_t iy = 1;
    Z[ix]     = x;
    Z[iy]     = y;

    // create f: U -> Z and vectors used for derivative calculations
    ADFun<double> f(U, Z);
    CPPAD_TESTVECTOR(double) v( f.Domain() );
    CPPAD_TESTVECTOR(double) w( f.Range() );

    // check parameters flags
    ok &= ! f.Parameter(ix);
    ok &=   f.Parameter(iy);

    // check function values
    ok &= ( Z[ix] == 2. );
    ok &= ( Z[iy] == 3. );

    // forward computation of partials w.r.t. s
    v[is] = 1.;
    w     = f.Forward(1, v);
    ok &= ( w[ix] == 1. );   // dx/ds
    ok &= ( w[iy] == 0. );   // dy/ds

    return ok;
}

bool copy_ad(void)
{   bool ok = true;   // initialize test result flag
    using CppAD::AD;  // so can use AD in place of CppAD::AD

    // domain space vector
    size_t n = 1;
    CPPAD_TESTVECTOR(AD<double>) x(n);
    x[0]     = 2.;

    // declare independent variables and start tape recording
    CppAD::Independent(x);

    // create an AD<double> that does not depend on x
    AD<double> b = 3.;

    // use copy constructor
    AD<double> u(x[0]);
    AD<double> v = b;

    // check which are parameters
    ok &= Variable(u);
    ok &= Parameter(v);

    // range space vector
    size_t m = 2;
    CPPAD_TESTVECTOR(AD<double>) y(m);
    y[0]  = u;
    y[1]  = v;

    // create f: x -> y and vectors used for derivative calculations
    CppAD::ADFun<double> f(x, y);
    CPPAD_TESTVECTOR(double) dx(n);
    CPPAD_TESTVECTOR(double) dy(m);

    // check parameters flags
    ok &= ! f.Parameter(0);
    ok &=   f.Parameter(1);

    // check function values
    ok &= ( y[0] == 2. );
    ok &= ( y[1] == 3. );

    // forward computation of partials w.r.t. x[0]
    dx[0] = 1.;
    dy    = f.Forward(1, dx);
    ok   &= ( dy[0] == 1. );   // du / dx
    ok   &= ( dy[1] == 0. );   // dv / dx

    return ok;
}
bool copy_base(void)
{   bool ok = true;    // initialize test result flag
    using CppAD::AD;   // so can use AD in place of CppAD::AD

    // construct directly from Base where Base is double
    AD<double> x(1.);

    // construct from a type that converts to Base where Base is double
    AD<double> y = 2;

    // construct from a type that converts to Base where Base = AD<double>
    AD< AD<double> > z(3);

    // check that resulting objects are parameters
    ok &= Parameter(x);
    ok &= Parameter(y);
    ok &= Parameter(z);

    // check values of objects (compare AD<double> with double)
    ok &= ( x == 1.);
    ok &= ( y == 2.);
    ok &= ( Value(z) == 3.);

    // user constructor through the static_cast template function
    x   = static_cast < AD<double> >( 4 );
    z  = static_cast < AD< AD<double> > >( 5 );

    ok &= ( x == 4. );
    ok &= ( Value(z) == 5. );

    return ok;
}
bool default_ctor(void)
{   bool ok = true;
    using CppAD::AD;

    // default AD constructor
    AD<double> x, y;

    // check that they are parameters
    ok &= Parameter(x);
    ok &= Parameter(y);

    // assign them values
    x = 3.;
    y = 4.;

    // just check a simple operation
    ok &= (x + y == 7.);

    return ok;
}

// END PROGRAM
} // end empty namespace
bool copy(void)
{   bool ok = true;
    ok &= copy_older();
    ok &= copy_ad();
    ok &= copy_base();
    ok &= default_ctor();
    return ok;
}
