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
$begin sparse_jac_for.cpp$$
$spell
    Cpp
    Jacobian
$$

$section Computing Sparse Jacobian Using Forward Mode: Example and Test$$

$srcthisfile%0%// BEGIN C++%// END C++%1%$$

$end
*/
// BEGIN C++
# include <cppad/cppad.hpp>
bool sparse_jac_for(void)
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
    size_t n = 3;
    a_vector  a_x(n);
    for(size_t j = 0; j < n; j++)
        a_x[j] = AD<double> (0);
    //
    // declare independent variables and starting recording
    CppAD::Independent(a_x);
    //
    size_t m = 4;
    a_vector  a_y(m);
    a_y[0] = a_x[0] + a_x[2];
    a_y[1] = a_x[0] + a_x[2];
    a_y[2] = a_x[1] + a_x[2];
    a_y[3] = a_x[1] + a_x[2] * a_x[2] / 2.;
    //
    // create f: x -> y and stop tape recording
    CppAD::ADFun<double> f(a_x, a_y);
    //
    // new value for the independent variable vector
    d_vector x(n);
    for(size_t j = 0; j < n; j++)
        x[j] = double(j);
    /*
           [ 1 0 1   ]
    J(x) = [ 1 0 1   ]
           [ 0 1 1   ]
           [ 0 1 x_2 ]
    */
    d_vector check(m * n);
    //
    // column-major order values of J(x)
    size_t nnz = 8;
    s_vector check_row(nnz), check_col(nnz);
    d_vector check_val(nnz);
    for(size_t k = 0; k < nnz; k++)
    {   // check_val
        if( k < 7 )
            check_val[k] = 1.0;
        else
            check_val[k] = x[2];
        //
        // check_row and check_col
        check_row[k] = k;
        if( k < 2 )
            check_col[k] = 0;
        else if( k < 4 )
            check_col[k] = 1;
        else
        {   check_col[k] = 2;
            check_row[k] = k - 4;
        }
    }
    //
    // n by n identity matrix sparsity
    sparse_rc<s_vector> pattern_in;
    pattern_in.resize(n, n, n);
    for(size_t k = 0; k < n; k++)
        pattern_in.set(k, k, k);
    //
    // sparsity for J(x)
    bool transpose     = false;
    bool dependency    = false;
    bool internal_bool = true;
    sparse_rc<s_vector> pattern_jac;
    f.for_jac_sparsity(
        pattern_in, transpose, dependency, internal_bool, pattern_jac
    );
    //
    // compute entire forward mode Jacobian
    sparse_rcv<s_vector, d_vector> subset( pattern_jac );
    CppAD::sparse_jac_work work;
    std::string coloring = "cppad";
    size_t group_max = 10;
    size_t n_color = f.sparse_jac_for(
        group_max, x, subset, pattern_jac, coloring, work
    );
    ok &= n_color == 2;
    //
    const s_vector row( subset.row() );
    const s_vector col( subset.col() );
    const d_vector val( subset.val() );
    s_vector col_major = subset.col_major();
    ok  &= subset.nnz() == nnz;
    for(size_t k = 0; k < nnz; k++)
    {   ok &= row[ col_major[k] ] == check_row[k];
        ok &= col[ col_major[k] ] == check_col[k];
        ok &= val[ col_major[k] ] == check_val[k];
    }
    // compute non-zero in row 3 only
    sparse_rc<s_vector> pattern_row3;
    pattern_row3.resize(m, n, 2); // nr = m, nc = n, nnz = 2
    pattern_row3.set(0, 3, 1);    // row[0] = 3, col[0] = 1
    pattern_row3.set(1, 3, 2);    // row[1] = 3, col[1] = 2
    sparse_rcv<s_vector, d_vector> subset_row3( pattern_row3 );
    work.clear();
    n_color = f.sparse_jac_for(
        group_max, x, subset_row3, pattern_jac, coloring, work
    );
    ok &= n_color == 2;
    //
    const s_vector row_row3( subset_row3.row() );
    const s_vector col_row3( subset_row3.col() );
    const d_vector val_row3( subset_row3.val() );
    ok &= subset_row3.nnz() == 2;
    //
    ok &= row_row3[0] == 3;
    ok &= col_row3[0] == 1;
    ok &= val_row3[0] == 1.0;
    //
    ok &= row_row3[1] == 3;
    ok &= col_row3[1] == 2;
    ok &= val_row3[1] == x[2];
    //
    return ok;
}
// END C++
