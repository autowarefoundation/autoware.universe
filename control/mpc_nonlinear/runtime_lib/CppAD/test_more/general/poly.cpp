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
// Old GetStarted example now used just for validation testing
*/
// BEGIN C++

// directory where cppad/cppad.hpp is stored must be searched by compiler
# include <cppad/cppad.hpp>

bool Poly(void)
{   bool ok = true;

    // make CppAD routines visible without CppAD:: infront of names
    using namespace CppAD;

    // degree of the polynomial that we will differentiate
    size_t deg = 4;

    // vector that will hold polynomial coefficients for p(z)
    CPPAD_TESTVECTOR(AD<double>) A(deg + 1);  // AD<double> elements
    CPPAD_TESTVECTOR(double)       a(deg + 1);  //    double  elements

    // set the polynomial coefficients
    A[0] = 1.;
    size_t k;
    for(k = 1; k <= deg; k++)
        A[k] = a[k] = 1.;

    // independent variables
    CPPAD_TESTVECTOR(AD<double>) Z(1); // one independent variable
    Z[0]     = 3.;                        // value of independent variable
    Independent(Z);                       // declare independent variable

    // dependent variables
    CPPAD_TESTVECTOR(AD<double>) P(1); // one dependent variable
    P[0]     = Poly(0, A, Z[0]);    // value of polynomial at Z[0]

    // define f : Z -> P as a function mapping independent to dependent
    ADFun<double> f(Z, P);          // ADFun corresponding to polynomial

    // compute derivative of polynomial
    CPPAD_TESTVECTOR(double) z(1);  // vector length f.Domain()
    CPPAD_TESTVECTOR(double) J(1);  // vector length f.Range * f.Domain()
    z[0] = 3.;                 // point at which to compute derivative
    J    = f.Jacobian(z);      // value of derivative

    // compare with derivative as computed by Poly
    ok  &= (Poly(1, a, z[0]) == J[0]);

    return ok;
}

// END C++
