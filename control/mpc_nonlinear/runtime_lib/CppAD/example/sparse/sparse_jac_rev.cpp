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
$begin sparse_jac_rev.cpp$$
$spell
    Cpp
    Jacobian
$$

$section Computing Sparse Jacobian Using Reverse Mode: Example and Test$$

$srcthisfile%0%// BEGIN C++%// END C++%1%$$

$end
*/
// BEGIN C++
# include <cppad/cppad.hpp>
bool sparse_jac_rev(void)
{   bool ok = true;
    //
    using CppAD::AD;
    using CppAD::NearEqual;
    using CppAD::sparse_rc;
    using CppAD::sparse_rcv;
    //
    typedef CPPAD_TESTVECTOR(AD<double>) a_vector;
    typedef CPPAD_TESTVECTOR(double)     d_vector;
    typedef CPPAD_TESTVECTOR(size_t)     s_vector;
    //
    // domain space vector
    size_t n = 4;
    a_vector  a_x(n);
    for(size_t j = 0; j < n; j++)
        a_x[j] = AD<double> (0);
    //
    // declare independent variables and starting recording
    CppAD::Independent(a_x);
    //
    size_t m = 3;
    a_vector  a_y(m);
    a_y[0] = a_x[0] + a_x[1];
    a_y[1] = a_x[2] + a_x[3];
    a_y[2] = a_x[0] + a_x[1] + a_x[2] + a_x[3] * a_x[3] / 2.;
    //
    // create f: x -> y and stop tape recording
    CppAD::ADFun<double> f(a_x, a_y);
    //
    // new value for the independent variable vector
    d_vector x(n);
    for(size_t j = 0; j < n; j++)
        x[j] = double(j);
    /*
           [ 1 1 0 0  ]
    J(x) = [ 0 0 1 1  ]
           [ 1 1 1 x_3]
    */
    //
    // row-major order values of J(x)
    size_t nnz = 8;
    s_vector check_row(nnz), check_col(nnz);
    d_vector check_val(nnz);
    for(size_t k = 0; k < nnz; k++)
    {   // check_val
        if( k < 7 )
            check_val[k] = 1.0;
        else
            check_val[k] = x[3];
        //
        // check_row and check_col
        check_col[k] = k;
        if( k < 2 )
            check_row[k] = 0;
        else if( k < 4 )
            check_row[k] = 1;
        else
        {   check_row[k] = 2;
            check_col[k] = k - 4;
        }
    }
    //
    // m by m identity matrix sparsity
    sparse_rc<s_vector> pattern_in(m, m, m);
    for(size_t k = 0; k < m; k++)
        pattern_in.set(k, k, k);
    //
    // sparsity for J(x)
    bool transpose     = false;
    bool dependency    = false;
    bool internal_bool = true;
    sparse_rc<s_vector> pattern_jac;
    f.rev_jac_sparsity(
        pattern_in, transpose, dependency, internal_bool, pattern_jac
    );
    //
    // compute entire reverse mode Jacobian
    sparse_rcv<s_vector, d_vector> subset( pattern_jac );
    CppAD::sparse_jac_work work;
    std::string coloring = "cppad";
    size_t n_sweep = f.sparse_jac_rev(x, subset, pattern_jac, coloring, work);
    ok &= n_sweep == 2;
    //
    const s_vector row( subset.row() );
    const s_vector col( subset.col() );
    const d_vector val( subset.val() );
    s_vector row_major = subset.row_major();
    ok  &= subset.nnz() == nnz;
    for(size_t k = 0; k < nnz; k++)
    {   ok &= row[ row_major[k] ] == check_row[k];
        ok &= col[ row_major[k] ] == check_col[k];
        ok &= val[ row_major[k] ] == check_val[k];
    }
    //
    // test using work stored by previous sparse_jac_rev
    sparse_rc<s_vector> pattern_not_used;
    std::string         coloring_not_used;
    n_sweep = f.sparse_jac_rev(x, subset, pattern_jac, coloring, work);
    ok &= n_sweep == 2;
    for(size_t k = 0; k < nnz; k++)
    {   ok &= row[ row_major[k] ] == check_row[k];
        ok &= col[ row_major[k] ] == check_col[k];
        ok &= val[ row_major[k] ] == check_val[k];
    }
    //
    // compute non-zero in col 3 only, nr = m, nc = n, nnz = 2
    sparse_rc<s_vector> pattern_col3(m, n, 2);
    pattern_col3.set(0, 1, 3);    // row[0] = 1, col[0] = 3
    pattern_col3.set(1, 2, 3);    // row[1] = 2, col[1] = 3
    sparse_rcv<s_vector, d_vector> subset_col3( pattern_col3 );
    work.clear();
    n_sweep = f.sparse_jac_rev(x, subset_col3, pattern_jac, coloring, work);
    ok &= n_sweep == 2;
    //
    const s_vector row_col3( subset_col3.row() );
    const s_vector col_col3( subset_col3.col() );
    const d_vector val_col3( subset_col3.val() );
    ok &= subset_col3.nnz() == 2;
    //
    ok &= row_col3[0] == 1;
    ok &= col_col3[0] == 3;
    ok &= val_col3[0] == 1.0;
    //
    ok &= row_col3[1] == 2;
    ok &= col_col3[1] == 3;
    ok &= val_col3[1] == x[3];
    //
    return ok;
}
// END C++
