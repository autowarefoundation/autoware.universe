/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-20 Bradley M. Bell

CppAD is distributed under the terms of the
             Eclipse Public License Version 2.0.

This Source Code may also be made available under the following
Secondary License when the conditions for such availability set forth
in the Eclipse Public License, Version 2.0 are satisfied:
      GNU General Public License, Version 2.0 or later.
---------------------------------------------------------------------------- */

/*
$begin complex_poly.cpp$$
$spell
    Cpp
$$

$section Complex Polynomial: Example and Test$$


$head Poly$$
Select this link to view specifications for $cref Poly$$:

$srcthisfile%0%// BEGIN C++%// END C++%1%$$

$end
*/
// BEGIN C++
// Complex examples should supppress conversion warnings
# include <cppad/wno_conversion.hpp>

# include <cppad/cppad.hpp>
# include <complex>

bool complex_poly(void)
{   bool ok    = true;
    size_t deg = 4;

    using CppAD::AD;
    using CppAD::Poly;
    typedef std::complex<double> Complex;

    // polynomial coefficients
    CPPAD_TESTVECTOR( Complex )   a   (deg + 1); // coefficients for p(z)
    CPPAD_TESTVECTOR(AD<Complex>) A   (deg + 1);
    size_t i;
    for(i = 0; i <= deg; i++)
        A[i] = a[i] = Complex(double(i), double(i));

    // independent variable vector
    CPPAD_TESTVECTOR(AD<Complex>) Z(1);
    Complex z = Complex(1., 2.);
    Z[0]      = z;
    Independent(Z);

    // dependent variable vector and indices
    CPPAD_TESTVECTOR(AD<Complex>) P(1);

    // dependent variable values
    P[0] = Poly(0, A, Z[0]);

    // create f: Z -> P and vectors used for derivative calculations
    CppAD::ADFun<Complex> f(Z, P);
    CPPAD_TESTVECTOR(Complex) v( f.Domain() );
    CPPAD_TESTVECTOR(Complex) w( f.Range() );

    // check first derivative w.r.t z
    v[0]      = 1.;
    w         = f.Forward(1, v);
    Complex p = Poly(1, a, z);
    ok &= ( w[0]  == p );

    // second derivative w.r.t z is 2 times its second order Taylor coeff
    v[0] = 0.;
    w    = f.Forward(2, v);
    p    = Poly(2, a, z);
    ok &= ( 2. * w[0]  == p );

    return ok;
}

// END C++
