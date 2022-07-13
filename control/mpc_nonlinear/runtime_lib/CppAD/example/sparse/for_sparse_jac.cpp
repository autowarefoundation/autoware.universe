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
$begin for_sparse_jac.cpp$$
$spell
    Jacobian
    Jac
    Dep
    Cpp
$$

$section Forward Mode Jacobian Sparsity: Example and Test$$


$srcthisfile%0%// BEGIN C++%// END C++%1%$$

$end
*/
// BEGIN C++

# include <set>
# include <cppad/cppad.hpp>

namespace { // -------------------------------------------------------------
// define the template function BoolCases<Vector>
template <class Vector>  // vector class, elements of type bool
bool BoolCases(void)
{   bool ok = true;
    using CppAD::AD;

    // domain space vector
    size_t n = 2;
    CPPAD_TESTVECTOR(AD<double>) X(n);
    X[0] = 0.;
    X[1] = 1.;

    // declare independent variables and start recording
    CppAD::Independent(X);

    // range space vector
    size_t m = 3;
    CPPAD_TESTVECTOR(AD<double>) Y(m);
    Y[0] = X[0];
    Y[1] = X[0] * X[1];
    Y[2] = X[1];

    // create f: X -> Y and stop tape recording
    CppAD::ADFun<double> f(X, Y);

    // sparsity pattern for the identity matrix
    Vector r(n * n);
    size_t i, j;
    for(i = 0; i < n; i++)
    {   for(j = 0; j < n; j++)
            r[ i * n + j ] = (i == j);
    }

    // sparsity pattern for F'(x)
    Vector s(m * n);
    s = f.ForSparseJac(n, r);

    // check values
    ok &= (s[ 0 * n + 0 ] == true);  // Y[0] does     depend on X[0]
    ok &= (s[ 0 * n + 1 ] == false); // Y[0] does not depend on X[1]
    ok &= (s[ 1 * n + 0 ] == true);  // Y[1] does     depend on X[0]
    ok &= (s[ 1 * n + 1 ] == true);  // Y[1] does     depend on X[1]
    ok &= (s[ 2 * n + 0 ] == false); // Y[2] does not depend on X[0]
    ok &= (s[ 2 * n + 1 ] == true);  // Y[2] does     depend on X[1]

    // check that values are stored
    ok &= (f.size_forward_bool() > 0);
    ok &= (f.size_forward_set() == 0);

    // sparsity pattern for F'(x)^T, note R is the identity, so R^T = R
    bool transpose = true;
    Vector st(n * m);
    st = f.ForSparseJac(n, r, transpose);

    // check values
    ok &= (st[ 0 * m + 0 ] == true);  // Y[0] does     depend on X[0]
    ok &= (st[ 1 * m + 0 ] == false); // Y[0] does not depend on X[1]
    ok &= (st[ 0 * m + 1 ] == true);  // Y[1] does     depend on X[0]
    ok &= (st[ 1 * m + 1 ] == true);  // Y[1] does     depend on X[1]
    ok &= (st[ 0 * m + 2 ] == false); // Y[2] does not depend on X[0]
    ok &= (st[ 1 * m + 2 ] == true);  // Y[2] does     depend on X[1]

    // check that values are stored
    ok &= (f.size_forward_bool() > 0);
    ok &= (f.size_forward_set() == 0);

    // free values from forward calculation
    f.size_forward_bool(0);
    ok &= (f.size_forward_bool() == 0);

    return ok;
}
// define the template function SetCases<Vector>
template <class Vector>  // vector class, elements of type std::set<size_t>
bool SetCases(void)
{   bool ok = true;
    using CppAD::AD;

    // domain space vector
    size_t n = 2;
    CPPAD_TESTVECTOR(AD<double>) X(n);
    X[0] = 0.;
    X[1] = 1.;

    // declare independent variables and start recording
    CppAD::Independent(X);

    // range space vector
    size_t m = 3;
    CPPAD_TESTVECTOR(AD<double>) Y(m);
    Y[0] = X[0];
    Y[1] = X[0] * X[1];
    Y[2] = X[1];

    // create f: X -> Y and stop tape recording
    CppAD::ADFun<double> f(X, Y);

    // sparsity pattern for the identity matrix
    Vector r(n);
    size_t i;
    for(i = 0; i < n; i++)
    {   assert( r[i].empty() );
        r[i].insert(i);
    }

    // sparsity pattern for F'(x)
    Vector s(m);
    s = f.ForSparseJac(n, r);

    // an interator to a standard set
    std::set<size_t>::iterator itr;
    bool found;

    // Y[0] does     depend on X[0]
    found = s[0].find(0) != s[0].end();  ok &= ( found == true );
    // Y[0] does not depend on X[1]
    found = s[0].find(1) != s[0].end();  ok &= ( found == false );
    // Y[1] does     depend on X[0]
    found = s[1].find(0) != s[1].end();  ok &= ( found == true );
    // Y[1] does     depend on X[1]
    found = s[1].find(1) != s[1].end();  ok &= ( found == true );
    // Y[2] does not depend on X[0]
    found = s[2].find(0) != s[2].end();  ok &= ( found == false );
    // Y[2] does     depend on X[1]
    found = s[2].find(1) != s[2].end();  ok &= ( found == true );

    // check that values are stored
    ok &= (f.size_forward_set() > 0);
    ok &= (f.size_forward_bool() == 0);


    // sparsity pattern for F'(x)^T
    bool transpose = true;
    Vector st(n);
    st = f.ForSparseJac(n, r, transpose);

    // Y[0] does     depend on X[0]
    found = st[0].find(0) != st[0].end();  ok &= ( found == true );
    // Y[0] does not depend on X[1]
    found = st[1].find(0) != st[1].end();  ok &= ( found == false );
    // Y[1] does     depend on X[0]
    found = st[0].find(1) != st[0].end();  ok &= ( found == true );
    // Y[1] does     depend on X[1]
    found = st[1].find(1) != st[1].end();  ok &= ( found == true );
    // Y[2] does not depend on X[0]
    found = st[0].find(2) != st[0].end();  ok &= ( found == false );
    // Y[2] does     depend on X[1]
    found = st[1].find(2) != st[1].end();  ok &= ( found == true );

    // check that values are stored
    ok &= (f.size_forward_set() > 0);
    ok &= (f.size_forward_bool() == 0);

    return ok;
}
} // End empty namespace
# include <vector>
# include <valarray>
bool ForSparseJac(void)
{   bool ok = true;
    // Run with Vector equal to four different cases
    // all of which are Simple Vectors with elements of type bool.
    ok &= BoolCases< CppAD::vectorBool     >();
    ok &= BoolCases< CppAD::vector  <bool> >();
    ok &= BoolCases< std::vector    <bool> >();
    ok &= BoolCases< std::valarray  <bool> >();

    // Run with Vector equal to two different cases both of which are
    // Simple Vectors with elements of type std::set<size_t>
    typedef std::set<size_t> set;
    ok &= SetCases< CppAD::vector  <set> >();
    // ok &= SetCases< std::vector    <set> >();

    // Do not use valarray because its element access in the const case
    // returns a copy instead of a reference
    // ok &= SetCases< std::valarray  <set> >();

    return ok;
}

// END C++
