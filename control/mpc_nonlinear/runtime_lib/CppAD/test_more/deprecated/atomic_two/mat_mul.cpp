/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-19 Bradley M. Bell

CppAD is distributed under the terms of the
             Eclipse Public License Version 2.0.

This Source Code may also be made available under the following
Secondary License when the conditions for such availability set forth
in the Eclipse Public License, Version 2.0 are satisfied:
      GNU General Public License, Version 2.0 or later.
---------------------------------------------------------------------------- */

/*
$begin atomic_two_mat_mul.cpp$$
$spell
    mul
$$

$section User Atomic Matrix Multiply: Example and Test$$

$head See Also$$
$cref atomic_two_eigen_mat_mul.cpp$$

$children%
    include/cppad/example/atomic_three/mat_mul.hpp
%$$

$head Class Definition$$
This example uses the file $cref atomic_three_mat_mul.hpp$$
which defines matrix multiply as a $cref atomic_two$$ operation.

$nospell

$head Use Atomic Function$$
$srccode%cpp% */
# include <cppad/cppad.hpp>
# include "mat_mul.hpp"

bool mat_mul(void)
{   bool ok = true;
    using CppAD::AD;
    using CppAD::vector;
    size_t i, j;
/* %$$
$subhead Constructor$$
$srccode%cpp% */
    // -------------------------------------------------------------------
    // object that multiplies  2 x 2  matrices
    atomic_mat_mul afun;
/* %$$
$subhead Recording$$
$srccode%cpp% */
    // start recording with four independent varables
    size_t n = 4;
    vector<double> x(n);
    vector< AD<double> > ax(n);
    for(j = 0; j < n; j++)
        ax[j] = x[j] = double(j + 1);
    CppAD::Independent(ax);

    // ------------------------------------------------------------------
    size_t nr_left = 2;
    size_t n_middle  = 2;
    size_t nc_right = 2;
    vector< AD<double> > atom_x(3 + (nr_left + nc_right) * n_middle );

    // matrix dimensions
    atom_x[0] = AD<double>( nr_left );
    atom_x[1] = AD<double>( n_middle );
    atom_x[2] = AD<double>( nc_right );

    // left matrix
    atom_x[3] = ax[0];  // left[0, 0] = x0
    atom_x[4] = ax[1];  // left[0, 1] = x1
    atom_x[5] = 5.;     // left[1, 0] = 5
    atom_x[6] = 6.;     // left[1, 1] = 6

    // right matix
    atom_x[7] = ax[2];  // right[0, 0] = x2
    atom_x[8] = 7.;     // right[0, 1] = 7
    atom_x[9] = ax[3];  // right[1, 0] = x3
    atom_x[10] = 8.;     // right[1, 1] = 8
    // ------------------------------------------------------------------
    /*
    [ x0 , x1 ] * [ x2 , 7 ] = [ x0*x2 + x1*x3 , x0*7 + x1*8 ]
    [ 5  , 6  ]   [ x3 , 8 ]   [  5*x2 +  6*x3 ,  5*7 +  6*8 ]
    */
    vector< AD<double> > atom_y(nr_left * nc_right);
    afun(atom_x, atom_y);

    ok &= (atom_y[0] == x[0]*x[2] + x[1]*x[3]) & Variable(atom_y[0]);
    ok &= (atom_y[1] == x[0]*7.   + x[1]*8.  ) & Variable(atom_y[1]);
    ok &= (atom_y[2] ==   5.*x[2] +   6.*x[3]) & Variable(atom_y[2]);
    ok &= (atom_y[3] ==   5.*7.   +   6.*8.  ) & Parameter(atom_y[3]);

    // ------------------------------------------------------------------
    // define the function g : x -> atom_y
    // g(x) = [ x0*x2 + x1*x3 , x0*7 + x1*8 , 5*x2  + 6*x3  , 5*7 + 6*8 ]^T
    CppAD::ADFun<double> g(ax, atom_y);
/* %$$
$subhead forward$$
$srccode%cpp% */
    // Test zero order forward mode evaluation of g(x)
    size_t m = atom_y.size();
    vector<double> y(m);
    for(j = 0; j <  n; j++)
        x[j] = double(j + 2);
    y = g.Forward(0, x);
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
    CppAD::vector<double> dx(n), dy(m);
    for(j = 0; j <  n; j++)
        dx[j] = double(j + 1);
    dy = g.Forward(1, dx);
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
    CppAD::vector<double> ddx(n), ddy(m);
    for(j = 0; j <  n; j++)
        ddx[j] = 0.;
    ddy = g.Forward(2, ddx);

    // [1, 2, 3, 4] * g_0^2 (x) * [1, 2, 3, 4]^T = 1*3 + 2*4 + 3*1 + 4*2
    ok &= 2. * ddy[0] == 1. * 3. + 2. * 4. + 3. * 1. + 4. * 2.;

    // for i > 0, [1, 2, 3, 4] * g_i^2 (x) * [1, 2, 3, 4]^T = 0
    ok &= ddy[1] == 0.;
    ok &= ddy[2] == 0.;
    ok &= ddy[3] == 0.;
/* %$$
$subhead reverse$$
$srccode%cpp% */
    // Test second order reverse mode
    CppAD::vector<double> w(m), dw(2 * n);
    for(i = 0; i < m; i++)
        w[i] = 0.;
    w[0] = 1.;
    dw = g.Reverse(2, w);

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
/* %$$
$subhead option$$
$srccode%cpp% */
    //----------------------------------------------------------------------
    // Test both the boolean and set sparsity at the atomic level
    for(size_t sparse_index = 0; sparse_index < 2; sparse_index++)
    {   if( sparse_index == 0 )
            afun.option( CppAD::atomic_base<double>::bool_sparsity_enum );
        else
            afun.option( CppAD::atomic_base<double>::set_sparsity_enum );
/* %$$
$subhead for_sparse_jac$$
$srccode%cpp% */
    // Test forward Jacobian sparsity pattern
    /*
    g(x) = [ x0*x2 + x1*x3 , x0*7 + x1*8 , 5*x2  + 6*x3  , 5*7 + 6*8 ]^T
    so the sparsity pattern should be
    s[0] = {0, 1, 2, 3}
    s[1] = {0, 1}
    s[2] = {2, 3}
    s[3] = {}
    */
    CppAD::vector< std::set<size_t> > r(n), s(m);
    for(j = 0; j <  n; j++)
    {   assert( r[j].empty() );
        r[j].insert(j);
    }
    s = g.ForSparseJac(n, r);
    for(j = 0; j <  n; j++)
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
/* %$$
$subhead rev_sparse_jac$$
$srccode%cpp% */
    // Test reverse Jacobian sparsity pattern
    for(i = 0; i <  m; i++)
    {   s[i].clear();
        s[i].insert(i);
    }
    r = g.RevSparseJac(m, s);
    for(j = 0; j <  n ; j++)
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
/* %$$
$subhead rev_sparse_hes$$
$srccode%cpp% */
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
    CppAD::vector< std::set<size_t> > h(n), t(1);
    t[0].clear();
    t[0].insert(0);
    h = g.RevSparseHes(n, t);
    size_t check[] = {2, 3, 0, 1};
    for(j = 0; j <  n; j++)
    {   // h[j] = { check[j] }
        for(i = 0; i < n; i++)
        {   if( i == check[j] )
                ok &= h[j].find(i) != h[j].end();
            else
                ok &= h[j].find(i) == h[j].end();
        }
    }
    t[0].clear();
    for( j = 1; j < n; j++)
            t[0].insert(j);
    h = g.RevSparseHes(n, t);
    for(j = 0; j <  n; j++)
    {   // h[j] = { }
        for(i = 0; i < n; i++)
            ok &= h[j].find(i) == h[j].end();
    }

    //-----------------------------------------------------------------
    } // end for(size_t sparse_index  ...
    //-----------------------------------------------------------------

    return ok;
}
/* %$$
$$ $comment end nospell$$
$end
*/
