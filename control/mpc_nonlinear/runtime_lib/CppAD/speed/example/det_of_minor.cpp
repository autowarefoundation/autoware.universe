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
$begin det_of_minor.cpp$$
$spell
    det
    Cpp
$$

$section Determinant of a Minor: Example and Test$$


$srcthisfile%0%// BEGIN C++%// END C++%1%$$

$end
*/
// BEGIN C++
# include <vector>
# include <cstddef>
# include <cppad/speed/det_of_minor.hpp>

bool det_of_minor()
{   bool   ok = true;
    size_t i;

    // dimension of the matrix A
    size_t m = 3;
    // index vectors set so minor is the entire matrix A
    std::vector<size_t> r(m + 1);
    std::vector<size_t> c(m + 1);
    for(i= 0; i < m; i++)
    {   r[i] = i+1;
        c[i] = i+1;
    }
    r[m] = 0;
    c[m] = 0;
    // values in the matrix A
    double  data[] = {
        1., 2., 3.,
        3., 2., 1.,
        2., 1., 2.
    };
    // construct vector a with the values of the matrix A
    std::vector<double> a(data, data + 9);

    // evaluate the determinant of A
    size_t n   = m; // minor has same dimension as A
    double det = CppAD::det_of_minor(a, m, n, r, c);

    // check the value of the determinant of A
    ok &= (det == (double) (1*(2*2-1*1) - 2*(3*2-1*2) + 3*(3*1-2*2)) );

    // minor where row 0 and column 1 are removed
    r[m] = 1;  // skip row index 0 by starting at row index 1
    c[0] = 2;  // skip column index 1 by pointing from index 0 to index 2
    // evaluate determinant of the minor
    n   = m - 1; // dimension of the minor
    det = CppAD::det_of_minor(a, m, m-1, r, c);

    // check the value of the determinant of the minor
    ok &= (det == (double) (3*2-1*2) );

    return ok;
}
// END C++
