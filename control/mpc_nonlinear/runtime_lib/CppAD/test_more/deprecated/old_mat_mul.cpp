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
$begin old_mat_mul.cpp$$
$spell
    mul
$$

$section Old Matrix Multiply as a User Atomic Operation: Example and Test$$

$head Deprecated 2013-05-27$$
This example has been deprecated;
use $cref atomic_two_mat_mul.cpp$$ instead.

$children%
    example/deprecated/old_mat_mul.hpp
%$$
$head Include File$$
This routine uses the include file old_mat_mul.hpp.

$srcthisfile%0%// BEGIN C++%// END C++%1%$$

$end
*/
// BEGIN C++
# include <cppad/cppad.hpp>
# include "old_mat_mul.hpp"

bool old_mat_mul(void)
{   bool ok = true;
    using CppAD::AD;

    // matrix sizes for this test
    size_t nr_result = 2;
    size_t n_middle  = 2;
    size_t nc_result = 2;

    // declare the AD<double> vectors ax and ay and X
    size_t n = nr_result * n_middle + n_middle * nc_result;
    size_t m = nr_result * nc_result;
    CppAD::vector< AD<double> > X(4), ax(n), ay(m);
    size_t i, j;
    for(j = 0; j < X.size(); j++)
        X[j] = (j + 1);

    // X is the vector of independent variables
    CppAD::Independent(X);
    // left matrix
    ax[0]  = X[0];  // left[0,0]   = x[0] = 1
    ax[1]  = X[1];  // left[0,1]   = x[1] = 2
    ax[2]  = 5.;    // left[1,0]   = 5
    ax[3]  = 6.;    // left[1,1]   = 6
    // right matrix
    ax[4]  = X[2];  // right[0,0]  = x[2] = 3
    ax[5]  = 7.;    // right[0,1]  = 7
    ax[6]  = X[3];  // right[1,0]  = x[3] = 4
    ax[7]  = 8.;    // right[1,1]  = 8
    /*
    [ x0 , x1 ] * [ x2 , 7 ] = [ x0*x2 + x1*x3 , x0*7 + x1*8 ]
    [ 5  , 6 ]    [ x3 , 8 ]   [ 5*x2  + 6*x3  , 5*7 + 6*8 ]
    */

    // The call back routines need to know the dimensions of the matrices.
    // Store information about the matrix multiply for this call to mat_mul.
    call_info info;
    info.nr_result = nr_result;
    info.n_middle  = n_middle;
    info.nc_result = nc_result;
    // info.vx gets set by forward during call to mat_mul below
    assert( info.vx.size() == 0 );
    size_t id      = info_.size();
    info_.push_back(info);

    // user defined AD<double> version of matrix multiply
    mat_mul(id, ax, ay);
    //----------------------------------------------------------------------
    // check AD<double>  results
    ok &= ay[0] == (1*3 + 2*4); ok &= Variable( ay[0] );
    ok &= ay[1] == (1*7 + 2*8); ok &= Variable( ay[1] );
    ok &= ay[2] == (5*3 + 6*4); ok &= Variable( ay[2] );
    ok &= ay[3] == (5*7 + 6*8); ok &= Parameter( ay[3] );
    //----------------------------------------------------------------------
    // use mat_mul to define a function g : X -> ay
    CppAD::ADFun<double> G;
    G.Dependent(X, ay);
    // g(x) = [ x0*x2 + x1*x3 , x0*7 + x1*8 , 5*x2  + 6*x3  , 5*7 + 6*8 ]^T
    //----------------------------------------------------------------------
    // Test zero order forward mode evaluation of g(x)
    CppAD::vector<double> x( X.size() ), y(m);
    for(j = 0; j <  X.size() ; j++)
        x[j] = double(j + 2);
    y = G.Forward(0, x);
    ok &= y[0] == x[0] * x[2] + x[1] * x[3];
    ok &= y[1] == x[0] * 7.   + x[1] * 8.;
    ok &= y[2] == 5. * x[2]   + 6. * x[3];
    ok &= y[3] == 5. * 7.     + 6. * 8.;

    //----------------------------------------------------------------------
    // Test first order forward mode evaluation of g'(x) * [1, 2, 3, 4]^T
    // g'(x) = [ x2, x3, x0, x1 ]
    //         [ 7 ,  8,  0, 0  ]
    //         [ 0 ,  0,  5, 6  ]
    //         [ 0 ,  0,  0, 0  ]
    CppAD::vector<double> dx( X.size() ), dy(m);
    for(j = 0; j <  X.size() ; j++)
        dx[j] = double(j + 1);
    dy = G.Forward(1, dx);
    ok &= dy[0] == 1. * x[2] + 2. * x[3] + 3. * x[0] + 4. * x[1];
    ok &= dy[1] == 1. * 7.   + 2. * 8.   + 3. * 0.   + 4. * 0.;
    ok &= dy[2] == 1. * 0.   + 2. * 0.   + 3. * 5.   + 4. * 6.;
    ok &= dy[3] == 1. * 0.   + 2. * 0.   + 3. * 0.   + 4. * 0.;

    //----------------------------------------------------------------------
    // Test second order forward mode
    // g_0^2 (x) = [ 0, 0, 1, 0 ], g_0^2 (x) * [1] = [3]
    //             [ 0, 0, 0, 1 ]              [2]   [4]
    //             [ 1, 0, 0, 0 ]              [3]   [1]
    //             [ 0, 1, 0, 0 ]              [4]   [2]
    CppAD::vector<double> ddx( X.size() ), ddy(m);
    for(j = 0; j <  X.size() ; j++)
        ddx[j] = 0.;
    ddy = G.Forward(2, ddx);
    // [1, 2, 3, 4] * g_0^2 (x) * [1, 2, 3, 4]^T = 1*3 + 2*4 + 3*1 + 4*2
    ok &= 2. * ddy[0] == 1. * 3. + 2. * 4. + 3. * 1. + 4. * 2.;
    // for i > 0, [1, 2, 3, 4] * g_i^2 (x) * [1, 2, 3, 4]^T = 0
    ok &= ddy[1] == 0.;
    ok &= ddy[2] == 0.;
    ok &= ddy[3] == 0.;

    //----------------------------------------------------------------------
    // Test second order reverse mode
    CppAD::vector<double> w(m), dw(2 *  X.size() );
    for(i = 0; i < m; i++)
        w[i] = 0.;
    w[0] = 1.;
    dw = G.Reverse(2, w);
    // g_0'(x) = [ x2, x3, x0, x1 ]
    ok &= dw[0*2 + 0] == x[2];
    ok &= dw[1*2 + 0] == x[3];
    ok &= dw[2*2 + 0] == x[0];
    ok &= dw[3*2 + 0] == x[1];
    // g_0'(x)   * [1, 2, 3, 4]  = 1 * x2 + 2 * x3 + 3 * x0 + 4 * x1
    // g_0^2 (x) * [1, 2, 3, 4]  = [3, 4, 1, 2]
    ok &= dw[0*2 + 1] == 3.;
    ok &= dw[1*2 + 1] == 4.;
    ok &= dw[2*2 + 1] == 1.;
    ok &= dw[3*2 + 1] == 2.;

    //----------------------------------------------------------------------
    // Test forward and reverse Jacobian sparsity pattern
    /*
    [ x0 , x1 ] * [ x2 , 7 ] = [ x0*x2 + x1*x3 , x0*7 + x1*8 ]
    [ 5  , 6 ]    [ x3 , 8 ]   [ 5*x2  + 6*x3  , 5*7 + 6*8 ]
    so the sparsity pattern should be
    s[0] = {0, 1, 2, 3}
    s[1] = {0, 1}
    s[2] = {2, 3}
    s[3] = {}
    */
    CppAD::vector< std::set<size_t> > r( X.size() ), s(m);
    for(j = 0; j <  X.size() ; j++)
    {   assert( r[j].empty() );
        r[j].insert(j);
    }
    s = G.ForSparseJac( X.size() , r);
    for(j = 0; j <  X.size() ; j++)
    {   // s[0] = {0, 1, 2, 3}
        ok &= s[0].find(j) != s[0].end();
        // s[1] = {0, 1}
        if( j == 0 || j == 1 )
            ok &= s[1].find(j) != s[1].end();
        else
            ok &= s[1].find(j) == s[1].end();
        // s[2] = {2, 3}
        if( j == 2 || j == 3 )
            ok &= s[2].find(j) != s[2].end();
        else
            ok &= s[2].find(j) == s[2].end();
    }
    // s[3] == {}
    ok &= s[3].empty();

    //----------------------------------------------------------------------
    // Test reverse Jacobian sparsity pattern
    /*
    [ x0 , x1 ] * [ x2 , 7 ] = [ x0*x2 + x1*x3 , x0*7 + x1*8 ]
    [ 5  , 6 ]    [ x3 , 8 ]   [ 5*x2  + 6*x3  , 5*7 + 6*8 ]
    so the sparsity pattern should be
    r[0] = {0, 1, 2, 3}
    r[1] = {0, 1}
    r[2] = {2, 3}
    r[3] = {}
    */
    for(i = 0; i <  m; i++)
    {   s[i].clear();
        s[i].insert(i);
    }
    r = G.RevSparseJac(m, s);
    for(j = 0; j <  X.size() ; j++)
    {   // r[0] = {0, 1, 2, 3}
        ok &= r[0].find(j) != r[0].end();
        // r[1] = {0, 1}
        if( j == 0 || j == 1 )
            ok &= r[1].find(j) != r[1].end();
        else
            ok &= r[1].find(j) == r[1].end();
        // r[2] = {2, 3}
        if( j == 2 || j == 3 )
            ok &= r[2].find(j) != r[2].end();
        else
            ok &= r[2].find(j) == r[2].end();
    }
    // r[3] == {}
    ok &= r[3].empty();

    //----------------------------------------------------------------------
    /* Test reverse Hessian sparsity pattern
    g_0^2 (x) = [ 0, 0, 1, 0 ] and for i > 0, g_i^2 = 0
                [ 0, 0, 0, 1 ]
                [ 1, 0, 0, 0 ]
                [ 0, 1, 0, 0 ]
    so for the sparsity pattern for the first component of g is
    h[0] = {2}
    h[1] = {3}
    h[2] = {0}
    h[3] = {1}
    */
    CppAD::vector< std::set<size_t> > h( X.size() ), t(1);
    t[0].clear();
    t[0].insert(0);
    h = G.RevSparseHes(X.size() , t);
    size_t check[] = {2, 3, 0, 1};
    for(j = 0; j <  X.size() ; j++)
    {   // h[j] = { check[j] }
        for(i = 0; i < n; i++)
        {   if( i == check[j] )
                ok &= h[j].find(i) != h[j].end();
            else
                ok &= h[j].find(i) == h[j].end();
        }
    }
    t[0].clear();
    for( j = 1; j < X.size(); j++)
            t[0].insert(j);
    h = G.RevSparseHes(X.size() , t);
    for(j = 0; j <  X.size() ; j++)
    {   // h[j] = { }
        for(i = 0; i < X.size(); i++)
            ok &= h[j].find(i) == h[j].end();
    }

    // --------------------------------------------------------------------
    // Free temporary work space. (If there are future calls to
    // old_mat_mul they would create new temporary work space.)
    CppAD::user_atomic<double>::clear();
    info_.clear();

    return ok;
}
// END C++
