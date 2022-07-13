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
Test of directional derivative in AD< AD< double > > case.
*/

# include <cppad/cppad.hpp>


bool fabs(void)
{   // test if CppAD::abs uses if statement during forward computations
    bool ok = true;

    using CppAD::Independent;
    using CppAD::ADFun;
    typedef CppAD::AD<double>      ADdouble;
    typedef CppAD::AD< ADdouble > ADDdouble;

    // af(x) = |x|
    CPPAD_TESTVECTOR( ADDdouble ) aax(1), aay(1);
    aax[0] = ADDdouble(0.);
    CppAD::Independent(aax);
    aay[0] = fabs(aax[0]);
    CppAD::ADFun< ADdouble > af(aax, aay);

    // f(x) = |x|
    CPPAD_TESTVECTOR( ADdouble ) ax(1), ay(1);
    ax[0] = ADdouble(0.);
    CppAD::Independent(ax);
    ay    = af.Forward(0, ax);
    CppAD::ADFun<double> f(ax, ay);

    // compute derivative of af at a positive argument
    CPPAD_TESTVECTOR( ADdouble ) adx(1), ady(1);
    ax[0]  = 1.;
    ay     = af.Forward(0, ax);
    adx[0] = 1;
    ady    = af.Forward(1, adx);
    ok    &= (ady[0] == 1.);

    // compute derivative of af at a zero argument
    ax[0]  = 0.;
    ay     = af.Forward(0, ax);
    adx[0] = 1;
    ady    = af.Forward(1, adx);
    ok    &= (ady[0] == 0.);

    // compute derivative of f at zero argument
    CPPAD_TESTVECTOR(double) x(1), y(1), dx(1), dy(1);
    x[0]  = 0.;
    y     = f.Forward(0, x);
    dx[0] = 1;
    dy    = f.Forward(1, dx);
    ok    &= (dy[0] == 0.);

    // compute derivative of af at a negative argument
    x[0]  = -1.;
    y     = f.Forward(0, x);
    dx[0] = 1;
    dy    = f.Forward(1, dx);
    ok    &= (dy[0] == -1.);

    return ok;
}
