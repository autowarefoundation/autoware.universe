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
$begin det_by_minor.cpp$$
$spell
    Cpp
$$

$section Determinant Using Expansion by Minors: Example and Test$$

$srcthisfile%0%// BEGIN C++%// END C++%1%$$

$end
*/
// BEGIN C++

# include <cppad/cppad.hpp>
# include <cppad/speed/det_by_minor.hpp>

bool det_by_minor()
{   bool ok = true;

    // dimension of the matrix
    size_t n = 3;

    // construct the determinat object
    CppAD::det_by_minor<double> Det(n);

    double  a[] = {
        1., 2., 3.,  // a[0] a[1] a[2]
        3., 2., 1.,  // a[3] a[4] a[5]
        2., 1., 2.   // a[6] a[7] a[8]
    };
    CPPAD_TESTVECTOR(double) A(9);
    size_t i;
    for(i = 0; i < 9; i++)
        A[i] = a[i];


    // evaluate the determinant
    double det = Det(A);

    double check;
    check = a[0]*(a[4]*a[8] - a[5]*a[7])
          - a[1]*(a[3]*a[8] - a[5]*a[6])
          + a[2]*(a[3]*a[7] - a[4]*a[6]);

    ok = det == check;

    return ok;
}

// END C++
