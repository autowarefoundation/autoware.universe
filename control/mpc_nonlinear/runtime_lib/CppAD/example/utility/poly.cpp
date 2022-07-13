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
$begin poly.cpp$$

$section Polynomial Evaluation: Example and Test$$


$srcthisfile%0%// BEGIN C++%// END C++%1%$$

$end
*/
// BEGIN C++

# include <cppad/cppad.hpp>
# include <cmath>

bool poly(void)
{   bool ok = true;

    // degree of the polynomial
    size_t deg = 3;

    // set the polynomial coefficients
    CPPAD_TESTVECTOR(double)   a(deg + 1);
    size_t i;
    for(i = 0; i <= deg; i++)
        a[i] = 1.;

    // evaluate this polynomial
    size_t k = 0;
    double z = 2.;
    double p = CppAD::Poly(k, a, z);
    ok      &= (p == 1. + z + z*z + z*z*z);

    // evaluate derivative
    k = 1;
    p = CppAD::Poly(k, a, z);
    ok &= (p == 1 + 2.*z + 3.*z*z);

    return ok;
}

// END C++
