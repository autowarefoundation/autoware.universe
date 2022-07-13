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
$begin lu_solve.cpp$$
$spell
    Geq
    Cpp
    Lu
$$

$section LuSolve With Complex Arguments: Example and Test$$


$srcthisfile%0%// BEGIN C++%// END C++%1%$$

$end
*/

// BEGIN C++

# include <cppad/utility/lu_solve.hpp>       // for CppAD::LuSolve
# include <cppad/utility/near_equal.hpp>     // for CppAD::NearEqual
# include <cppad/utility/vector.hpp>  // for CppAD::vector
# include <complex>               // for std::complex

typedef std::complex<double> Complex;    // define the Complex type
bool LuSolve(void)
{   bool  ok = true;
    using namespace CppAD;

    size_t   n = 3;           // number rows in A and B
    size_t   m = 2;           // number columns in B, X and S

    // A is an n by n matrix, B, X, and S are n by m matrices
    CppAD::vector<Complex> A(n * n), B(n * m), X(n * m) , S(n * m);

    Complex  logdet;          // log of determinant of A
    int      signdet;         // zero if A is singular
    Complex  det;             // determinant of A
    size_t   i, j, k;         // some temporary indices

    // set A equal to the n by n Hilbert Matrix
    for(i = 0; i < n; i++)
        for(j = 0; j < n; j++)
            A[i * n + j] = 1. / (double) (i + j + 1);

    // set S to the solution of the equation we will solve
    for(j = 0; j < n; j++)
        for(k = 0; k < m; k++)
            S[ j * m + k ] = Complex(double(j), double(j + k));

    // set B = A * S
    size_t ik;
    Complex sum;
    for(k = 0; k < m; k++)
    {   for(i = 0; i < n; i++)
        {   sum = 0.;
            for(j = 0; j < n; j++)
                sum += A[i * n + j] * S[j * m + k];
            B[i * m + k] = sum;
        }
    }

    // solve the equation A * X = B and compute determinant of A
    signdet = CppAD::LuSolve(n, m, A, B, X, logdet);
    det     = Complex( signdet ) * exp( logdet );

    double cond  = 4.62963e-4;       // condition number of A when n = 3
    double determinant = 1. / 2160.; // determinant of A when n = 3
    double delta = 1e-14 / cond;     // accuracy expected in X

    // check determinant
    ok &= CppAD::NearEqual(det, determinant, delta, delta);

    // check solution
    for(ik = 0; ik < n * m; ik++)
        ok &= CppAD::NearEqual(X[ik], S[ik], delta, delta);

    return ok;
}
// END C++
