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
$begin sparse_jac_work.cpp$$
$spell
    Cpp
    Jacobian
$$

$section Using Same Work With Different Levels of AD: Example and Test$$

$srcthisfile%0%// BEGIN C++%// END C++%1%$$

$end
*/
// BEGIN C++
# include <cppad/cppad.hpp>
bool sparse_jac_work(void)
{   bool ok = true;
    //
    using CppAD::AD;
    using CppAD::NearEqual;
    using CppAD::sparse_rc;
    using CppAD::sparse_rcv;
    //
    typedef AD<double>                   a1double;
    typedef AD<a1double>                 a2double;
    typedef CPPAD_TESTVECTOR(double)     d_vector;
    typedef CPPAD_TESTVECTOR(a1double)   a1vector;
    typedef CPPAD_TESTVECTOR(a2double)   a2vector;
    typedef CPPAD_TESTVECTOR(size_t)     s_vector;
    //
    // domain space vector
    size_t n = 4;
    a2vector a2x(n);
    a1vector a1x(n);
    for(size_t j = 0; j < n; j++)
        a2x[j] = a1x[j] = 0.0;
    //
    // declare independent variables and starting recording
    CppAD::Independent(a2x);
    //
    size_t m = 3;
    a2vector  a2y(m);
    a2y[0] = a2x[0] + a2x[1];
    a2y[1] = a2x[2] + a2x[3];
    a2y[2] = a2x[0] + a2x[1] + a2x[2] + a2x[3] * a2x[3] / 2.;
    //
    // create f: x -> y for computation with a1double
    CppAD::ADFun<a1double> a1f(a2x, a2y);
    //
    // create an identical function for computation with double
    CppAD::Independent(a1x);
    a1vector a1y = a1f.Forward(0, a1x);
    CppAD::ADFun<double> f(a1x, a1y);
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
    // test using work stored by previous sparse_jac_rev with f
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
    // test using work stored by previous sparse_jac_rev with a1f
    sparse_rcv<s_vector, a1vector> a1subset( pattern_jac );
    for(size_t j = 0; j < n; ++j)
        a1x[j] = x[j];
    n_sweep = a1f.sparse_jac_rev(a1x, a1subset, pattern_jac, coloring, work);
    ok &= n_sweep == 2;
    const a1vector a1val( a1subset.val() );
    for(size_t k = 0; k < nnz; k++)
    {   ok &= row[ row_major[k] ] == check_row[k];
        ok &= col[ row_major[k] ] == check_col[k];
        ok &= Value( a1val[ row_major[k] ] ) == check_val[k];
    }
    //
    return ok;
}
// END C++
