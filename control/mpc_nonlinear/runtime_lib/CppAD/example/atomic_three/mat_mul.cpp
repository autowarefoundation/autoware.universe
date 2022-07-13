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
$begin atomic_three_mat_mul.cpp$$
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
which defines matrix multiply as a $cref atomic_three$$ operation.

$nospell

$head Use Atomic Function$$
$srccode%cpp% */
# include <cppad/cppad.hpp>
# include <cppad/example/atomic_three/mat_mul.hpp>

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
    size_t nr_left   = 2;
    size_t n_middle  = 2;
    size_t nc_right  = 2;
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
$subhead jac_sparsity$$
$srccode%cpp% */
    // sparsity pattern for the Jacobian
    // g'(x) = [ x2, x3, x0, x1  ]
    //         [  7,  8,  0,  0  ]
    //         [  0,  0,  5,  6  ]
    //         [  0,  0,  0,  0  ]
    CppAD::sparse_rc< CPPAD_TESTVECTOR(size_t) > pattern_in, pattern_out;
    bool transpose     = false;
    bool dependency    = false;
    bool internal_bool = false;
    // test both forward and reverse mode
    for(size_t forward_mode = 0; forward_mode <= 1; ++forward_mode)
    {   if( bool(forward_mode) )
        {   pattern_in.resize(n, n, n);
            for(j = 0; j < n; ++j)
                pattern_in.set(j, j, j);
            g.for_jac_sparsity(
                pattern_in, transpose, dependency, internal_bool, pattern_out
            );
        }
        else
        {   pattern_in.resize(m, m, m);
            for(i = 0; i < m; ++i)
                pattern_in.set(i, i, i);
            g.rev_jac_sparsity(
                pattern_in, transpose, dependency, internal_bool, pattern_out
            );
        }
        const CPPAD_TESTVECTOR(size_t)& row = pattern_out.row();
        const CPPAD_TESTVECTOR(size_t)& col = pattern_out.col();
        CPPAD_TESTVECTOR(size_t) row_major  = pattern_out.row_major();
        size_t k = 0;
        for(j = 0; j < n; ++j)
        {   ok &= row[ row_major[k] ] == 0; // (0, j)
            ok &= col[ row_major[k] ] == j;
            ++k;
        }
        ok &= row[ row_major[k] ] == 1; // (1, 0)
        ok &= col[ row_major[k] ] == 0; //
        ++k;
        ok &= row[ row_major[k] ] == 1; // (1, 1)
        ok &= col[ row_major[k] ] == 1; //
        ++k;
        ok &= row[ row_major[k] ] == 2; // (2, 2)
        ok &= col[ row_major[k] ] == 2; //
        ++k;
        ok &= row[ row_major[k] ] == 2; // (2, 3)
        ok &= col[ row_major[k] ] == 3; //
        ++k;
        ok &= pattern_out.nnz() == k;
    }
/* %$$
$subhead hes_sparsity$$
$srccode%cpp% */
    /* Hessian sparsity pattern
    g_0^2 (x) = [ 0, 0, 1, 0 ] and for i > 0, g_i^2 = 0
                [ 0, 0, 0, 1 ]
                [ 1, 0, 0, 0 ]
                [ 0, 1, 0, 0 ]
    */
    CPPAD_TESTVECTOR(bool) select_x(n), select_y(m);
    for(j = 0; j < n; ++j)
        select_x[j] = true;
    for(i = 0; i < m; ++i)
        select_y[i] = true;
    for(size_t forward_mode = 0; forward_mode <= 1; ++forward_mode)
    {   if( bool(forward_mode) )
        {   g.for_hes_sparsity(
                select_y, select_x, internal_bool, pattern_out
            );
        }
        else
        {   // results for for_jac_sparsity are stored in g
            g.rev_hes_sparsity(
                select_y, transpose, internal_bool, pattern_out
            );
        }
        const CPPAD_TESTVECTOR(size_t)& row = pattern_out.row();
        const CPPAD_TESTVECTOR(size_t)& col = pattern_out.col();
        CPPAD_TESTVECTOR(size_t) row_major  = pattern_out.row_major();
        size_t k = 0;
        ok &= row[ row_major[k] ] == 0; // (0, 2)
        ok &= col[ row_major[k] ] == 2;
        ++k;
        ok &= row[ row_major[k] ] == 1; // (1, 3)
        ok &= col[ row_major[k] ] == 3;
        ++k;
        ok &= row[ row_major[k] ] == 2; // (2, 0)
        ok &= col[ row_major[k] ] == 0;
        ++k;
        ok &= row[ row_major[k] ] == 3; // (3, 1)
        ok &= col[ row_major[k] ] == 1;
        ++k;
        ok &= pattern_out.nnz() == k;
    }

    return ok;
}
/* %$$
$$ $comment end nospell$$
$end
*/
