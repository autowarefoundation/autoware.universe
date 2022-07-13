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
$begin for_sparse_hes.cpp$$
$spell
    Hessian
    Jac
    Hes
    Dep
    Cpp
$$

$section Forward Mode Hessian Sparsity: Example and Test$$


$srcthisfile%0%// BEGIN C++%// END C++%1%$$

$end
*/
// BEGIN C++

# include <cppad/cppad.hpp>
namespace { // -------------------------------------------------------------

// expected sparsity pattern
bool check_f0[] = {
    false, false, false,  // partials w.r.t x0 and (x0, x1, x2)
    false, false, false,  // partials w.r.t x1 and (x0, x1, x2)
    false, false, true    // partials w.r.t x2 and (x0, x1, x2)
};
bool check_f1[] = {
    false,  true, false,  // partials w.r.t x0 and (x0, x1, x2)
    true,  false, false,  // partials w.r.t x1 and (x0, x1, x2)
    false, false, false   // partials w.r.t x2 and (x0, x1, x2)
};

// define the template function BoolCases<Vector> in empty namespace
template <class Vector> // vector class, elements of type bool
bool BoolCases(bool optimize)
{   bool ok = true;
    using CppAD::AD;

    // domain space vector
    size_t n = 3;
    CPPAD_TESTVECTOR(AD<double>) ax(n);
    ax[0] = 0.;
    ax[1] = 1.;
    ax[2] = 2.;

    // declare independent variables and start recording
    CppAD::Independent(ax);

    // range space vector
    size_t m = 2;
    CPPAD_TESTVECTOR(AD<double>) ay(m);
    ay[0] = sin( ax[2] ) + ax[0] + ax[1] + ax[2];
    ay[1] = ax[0] * ax[1];

    // create f: x -> y and stop tape recording
    CppAD::ADFun<double> f(ax, ay);
    if( optimize )
        f.optimize();

    // sparsity pattern for diagonal of identity matrix
    Vector r(n);
    size_t i, j;
    for(i = 0; i < n; i++)
        r[ i ] = true;

    // compute sparsity pattern for H(x) = F_0^{(2)} (x)
    Vector s(m);
    for(i = 0; i < m; i++)
        s[i] = false;
    s[0] = true;
    Vector h(n * n);
    h    = f.ForSparseHes(r, s);

    // check values
    for(i = 0; i < n; i++)
        for(j = 0; j < n; j++)
            ok &= (h[ i * n + j ] == check_f0[ i * n + j ] );

    // compute sparsity pattern for H(x) = F_1^{(2)} (x)
    for(i = 0; i < m; i++)
        s[i] = false;
    s[1] = true;
    h    = f.ForSparseHes(r, s);

    // check values
    for(i = 0; i < n; i++)
        for(j = 0; j < n; j++)
            ok &= (h[ i * n + j ] == check_f1[ i * n + j ] );

    return ok;
}
// define the template function SetCases<Vector> in empty namespace
template <class Vector> // vector class, elements of type std::set<size_t>
bool SetCases(bool optimize)
{   bool ok = true;
    using CppAD::AD;

    // domain space vector
    size_t n = 3;
    CPPAD_TESTVECTOR(AD<double>) ax(n);
    ax[0] = 0.;
    ax[1] = 1.;
    ax[2] = 2.;

    // declare independent variables and start recording
    CppAD::Independent(ax);

    // range space vector
    size_t m = 2;
    CPPAD_TESTVECTOR(AD<double>) ay(m);
    ay[0] = sin( ax[2] );
    ay[1] = ax[0] * ax[1];

    // create f: x -> y and stop tape recording
    CppAD::ADFun<double> f(ax, ay);
    if( optimize )
        f.optimize();

    // sparsity pattern for the diagonal of the identity matrix
    Vector r(1);
    size_t i;
    for(i = 0; i < n; i++)
        r[0].insert(i);

    // compute sparsity pattern for H(x) = F_0^{(2)} (x)
    Vector s(1);
    assert( s[0].empty() );
    s[0].insert(0);
    Vector h(n);
    h    = f.ForSparseHes(r, s);

    // check values
    std::set<size_t>::iterator itr;
    size_t j;
    for(i = 0; i < n; i++)
    {   for(j = 0; j < n; j++)
        {   bool found = h[i].find(j) != h[i].end();
            ok        &= (found == check_f0[i * n + j]);
        }
    }

    // compute sparsity pattern for H(x) = F_1^{(2)} (x)
    s[0].clear();
    assert( s[0].empty() );
    s[0].insert(1);
    h    = f.ForSparseHes(r, s);

    // check values
    for(i = 0; i < n; i++)
    {   for(j = 0; j < n; j++)
        {   bool found = h[i].find(j) != h[i].end();
            ok        &= (found == check_f1[i * n + j]);
        }
    }

    return ok;
}
} // End empty namespace

# include <vector>
# include <valarray>
bool for_sparse_hes(void)
{   bool ok = true;
    for(size_t k = 0; k < 2; k++)
    {   bool optimize = bool(k);

        // Run with Vector equal to four different cases
        // all of which are Simple Vectors with elements of type bool.
        ok &= BoolCases< CppAD::vector  <bool> >(optimize);
        ok &= BoolCases< CppAD::vectorBool     >(optimize);
        ok &= BoolCases< std::vector    <bool> >(optimize);
        ok &= BoolCases< std::valarray  <bool> >(optimize);

        // Run with Vector equal to two different cases both of which are
        // Simple Vectors with elements of type std::set<size_t>
        typedef std::set<size_t> set;
        ok &= SetCases< CppAD::vector  <set> >(optimize);
        ok &= SetCases< std::vector    <set> >(optimize);

        // Do not use valarray because its element access in the const case
        // returns a copy instead of a reference
        // ok &= SetCases< std::valarray  <set> >(optimize);
    }
    return ok;
}


// END C++
