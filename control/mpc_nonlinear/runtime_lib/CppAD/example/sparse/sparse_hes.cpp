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
$begin sparse_hes.cpp$$
$spell
    Cpp
    Hessian
$$

$section Computing Sparse Hessian: Example and Test$$


$srcthisfile%0%// BEGIN C++%// END C++%1%$$

$end
*/
// BEGIN C++
# include <cppad/cppad.hpp>
bool sparse_hes(void)
{   bool ok = true;
    using CppAD::AD;
    using CppAD::NearEqual;
    //
    typedef CPPAD_TESTVECTOR(AD<double>)               a_vector;
    typedef CPPAD_TESTVECTOR(double)                   d_vector;
    typedef CPPAD_TESTVECTOR(size_t)                   s_vector;
    typedef CPPAD_TESTVECTOR(bool)                     b_vector;
    //
    // domain space vector
    size_t n = 12;  // must be greater than or equal 3; see n_sweep below
    a_vector a_x(n);
    for(size_t j = 0; j < n; j++)
        a_x[j] = AD<double> (0);
    //
    // declare independent variables and starting recording
    CppAD::Independent(a_x);
    //
    // range space vector
    size_t m = 1;
    a_vector a_y(m);
    a_y[0] = a_x[0] * a_x[1];
    for(size_t j = 0; j < n; j++)
        a_y[0] += a_x[j] * a_x[j] * a_x[j];
    //
    // create f: x -> y and stop tape recording
    // (without executing zero order forward calculation)
    CppAD::ADFun<double> f;
    f.Dependent(a_x, a_y);
    //
    // new value for the independent variable vector, and weighting vector
    d_vector w(m), x(n);
    for(size_t j = 0; j < n; j++)
        x[j] = double(j);
    w[0] = 1.0;
    //
    // vector used to check the value of the hessian
    d_vector check(n * n);
    size_t ij  = 0 * n + 1;
    for(ij = 0; ij < n * n; ij++)
        check[ij] = 0.0;
    ij         = 0 * n + 1;
    check[ij]  = 1.0;
    ij         = 1 * n + 0;
    check[ij]  = 1.0 ;
    for(size_t j = 0; j < n; j++)
    {   ij = j * n + j;
        check[ij] = 6.0 * x[j];
    }
    //
    // compute Hessian sparsity pattern
    b_vector select_domain(n), select_range(m);
    for(size_t j = 0; j < n; j++)
        select_domain[j] = true;
    select_range[0] = true;
    //
    CppAD::sparse_rc<s_vector> hes_pattern;
    bool internal_bool = false;
    f.for_hes_sparsity(
        select_domain, select_range, internal_bool, hes_pattern
    );
    //
    // compute entire sparse Hessian (really only need lower triangle)
    CppAD::sparse_rcv<s_vector, d_vector> subset( hes_pattern );
    CppAD::sparse_hes_work work;
    std::string coloring = "cppad.symmetric";
    size_t n_sweep = f.sparse_hes(x, w, subset, hes_pattern, coloring, work);
    ok &= n_sweep == 2;
    //
    const s_vector row( subset.row() );
    const s_vector col( subset.col() );
    const d_vector val( subset.val() );
    size_t nnz = subset.nnz();
    ok &= nnz == n + 2;
    for(size_t k = 0; k < nnz; k++)
    {   ij = row[k] * n + col[k];
        ok &= val[k] == check[ij];
    }
    return ok;
}
// END C++
