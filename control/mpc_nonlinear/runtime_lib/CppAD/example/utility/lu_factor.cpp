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
$begin lu_factor.cpp$$
$spell
    Geq
    Cpp
    Lu
$$

$section LuFactor: Example and Test$$


$srcthisfile%0%// BEGIN C++%// END C++%1%$$

$end
*/

// BEGIN C++
# include <cstdlib>               // for rand function
# include <cppad/utility/lu_factor.hpp>      // for CppAD::LuFactor
# include <cppad/utility/near_equal.hpp>     // for CppAD::NearEqual
# include <cppad/utility/vector.hpp>  // for CppAD::vector

bool LuFactor(void)
{   bool  ok = true;

# ifndef _MSC_VER
    using std::rand;
    using std::srand;
# endif
    using CppAD::NearEqual;
    double eps99 = 99.0 * std::numeric_limits<double>::epsilon();

    size_t  n = 5;                        // number rows in A
    double  rand_max = double(RAND_MAX);  // maximum rand value
    double  sum;                          // element of L * U
    double  pij;                          // element of permuted A
    size_t  i, j, k;                      // temporary indices

    // A is an n by n matrix
    CppAD::vector<double> A(n*n), LU(n*n), L(n*n), U(n*n);

    // set A equal to an n by n random matrix
    for(i = 0; i < n; i++)
        for(j = 0; j < n; j++)
            A[i * n + j] = rand() / rand_max;

    // pivot vectors
    CppAD::vector<size_t> ip(n);
    CppAD::vector<size_t> jp(n);

    // factor the matrix A
    LU       = A;
    CppAD::LuFactor(ip, jp, LU);

    // check that ip and jp are permutations of the indices 0, ... , n-1
    for(i = 0; i < n; i++)
    {   ok &= (ip[i] < n);
        ok &= (jp[i] < n);
        for(j = 0; j < n; j++)
        {   if( i != j )
            {   ok &= (ip[i] != ip[j]);
                ok &= (jp[i] != jp[j]);
            }
        }
    }

    // Extract L from LU
    for(i = 0; i < n; i++)
    {   // elements along and below the diagonal
        for(j = 0; j <= i; j++)
            L[i * n + j] = LU[ ip[i] * n + jp[j] ];
        // elements above the diagonal
        for(j = i+1; j < n; j++)
            L[i * n + j] = 0.;
    }

    // Extract U from LU
    for(i = 0; i < n; i++)
    {   // elements below the diagonal
        for(j = 0; j < i; j++)
            U[i * n + j] = 0.;
        // elements along the diagonal
        U[i * n + i] = 1.;
        // elements above the diagonal
        for(j = i+1; j < n; j++)
            U[i * n + j] = LU[ ip[i] * n + jp[j] ];
    }

    // Compute L * U
    for(i = 0; i < n; i++)
    {   for(j = 0; j < n; j++)
        {   // compute element (i,j) entry in L * U
            sum = 0.;
            for(k = 0; k < n; k++)
                sum += L[i * n + k] * U[k * n + j];
            // element (i,j) in permuted version of A
            pij  = A[ ip[i] * n + jp[j] ];
            // compare
            ok  &= NearEqual(pij, sum, eps99, eps99);
        }
    }

    return ok;
}

// END C++
