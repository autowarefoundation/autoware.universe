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
Old example, now used just for validation testing
*/

# include <cppad/cppad.hpp>

bool assign(void)
{   bool ok = true;
    using CppAD::AD;

    // domain space vector
    size_t n = 3;
    CPPAD_TESTVECTOR(AD<double>) x(n);
    x[0]     = 2;      // AD<double> = int
    x[1]     = 3.;     // AD<double> = double
    x[2]     = x[1];   // AD<double> = AD<double>

    // declare independent variables and start tape recording
    CppAD::Independent(x);

    // range space vector
    size_t m = 3;
    CPPAD_TESTVECTOR(AD<double>) y(m);

    // assign an AD<Base> object equal to an independent variable
    // (choose the first independent variable to check a special case)
    // use the value returned by the assignment (for another assignment)
    y[0] = y[1] = x[0];

    // assign an AD<Base> object equal to an expression
    y[1] = x[1] + 1.;
    y[2] = x[2] + 2.;

    // check that all the resulting components of y depend on x
    ok &= Variable(y[0]);  // y[0] = x[0]
    ok &= Variable(y[1]);  // y[1] = x[1] + 1
    ok &= Variable(y[2]);  // y[2] = x[2] + 2

    // construct f : x -> y and stop the tape recording
    CppAD::ADFun<double> f(x, y);

    // check variable values
    ok &= ( y[0] == 2.);
    ok &= ( y[1] == 4.);
    ok &= ( y[2] == 5.);

    // compute partials w.r.t x[1]
    CPPAD_TESTVECTOR(double) dx(n);
    CPPAD_TESTVECTOR(double) dy(m);
    dx[0] = 0.;
    dx[1] = 1.;
    dx[2] = 0.;
    dy   = f.Forward(1, dx);
    ok  &= (dy[0] == 0.);  // dy[0] / dx[1]
    ok  &= (dy[1] == 1.);  // dy[1] / dx[1]
    ok  &= (dy[2] == 0.);  // dy[2] / dx[1]

    // compute the derivative y[2]
    CPPAD_TESTVECTOR(double)  w(m);
    CPPAD_TESTVECTOR(double) dw(n);
    w[0] = 0.;
    w[1] = 0.;
    w[2] = 1.;
    dw   = f.Reverse(1, w);
    ok  &= (dw[0] == 0.);  // dy[2] / dx[0]
    ok  &= (dw[1] == 0.);  // dy[2] / dx[1]
    ok  &= (dw[2] == 1.);  // dy[2] / dx[2]

    // assign a VecAD<Base>::reference
    CppAD::VecAD<double> v(1);
    AD<double> zero(0);
    v[zero] = 5.;
    ok     &= (v[0] == 5.);

    return ok;
}
