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
$begin sparse_sub_hes.cpp$$
$spell
$$

$section Subset of a Sparse Hessian: Example and Test$$

$head Purpose$$
This example uses a
$cref/column subset/sparse_hessian/p/Column Subset/$$ of the sparsity pattern
to compute a subset of the Hessian.

$head See Also$$
$cref sub_sparse_hes.cpp$$

$srcthisfile%0%// BEGIN C++%// END C++%1%$$

$end
*/
// BEGIN C++
# include <cppad/cppad.hpp>
bool sparse_sub_hes(void)
{   bool ok = true;
    using CppAD::AD;
    typedef CPPAD_TESTVECTOR(size_t)     SizeVector;
    typedef CPPAD_TESTVECTOR(double)     DoubleVector;
    typedef CppAD::sparse_rc<SizeVector> sparsity;
    //
    // domain space vector
    size_t n = 4;
    CPPAD_TESTVECTOR(AD<double>) ax(n);
    for(size_t j = 0; j < n; j++)
        ax[j] = double(j);

    // declare independent variables and start recording
    CppAD::Independent(ax);

    // range space vector
    size_t m = 1;
    CPPAD_TESTVECTOR(AD<double>) ay(m);
    ay[0] = 0.0;
    for(size_t j = 0; j < n; j++)
        ay[0] += double(j+1) * ax[0] * ax[j];

    // create f: x -> y and stop tape recording
    CppAD::ADFun<double> f(ax, ay);

    // sparsity pattern for the identity matrix
    size_t nr     = n;
    size_t nc     = n;
    size_t nnz_in = n;
    sparsity pattern_in(nr, nc, nnz_in);
    for(size_t k = 0; k < nnz_in; k++)
    {   size_t r = k;
        size_t c = k;
        pattern_in.set(k, r, c);
    }
    // compute sparsity pattern for J(x) = f'(x)
    bool transpose       = false;
    bool dependency      = false;
    bool internal_bool   = false;
    sparsity pattern_out;
    f.for_jac_sparsity(
        pattern_in, transpose, dependency, internal_bool, pattern_out
    );
    //
    // compute sparsity pattern for H(x) = f''(x)
    CPPAD_TESTVECTOR(bool) select_range(m);
    select_range[0]      = true;
    CppAD::sparse_hes_work work;
    f.rev_hes_sparsity(
        select_range, transpose, internal_bool, pattern_out
    );
    size_t nnz = pattern_out.nnz();
    ok        &= nnz == 7;
    ok        &= pattern_out.nr() == n;
    ok        &= pattern_out.nc() == n;
    {   // check results
        const SizeVector& row( pattern_out.row() );
        const SizeVector& col( pattern_out.col() );
        SizeVector row_major = pattern_out.row_major();
        //
        ok &= row[ row_major[0] ] ==  0  && col[ row_major[0] ] ==  0;
        ok &= row[ row_major[1] ] ==  0  && col[ row_major[1] ] ==  1;
        ok &= row[ row_major[2] ] ==  0  && col[ row_major[2] ] ==  2;
        ok &= row[ row_major[3] ] ==  0  && col[ row_major[3] ] ==  3;
        //
        ok &= row[ row_major[4] ] ==  1  && col[ row_major[4] ] ==  0;
        ok &= row[ row_major[5] ] ==  2  && col[ row_major[5] ] ==  0;
        ok &= row[ row_major[6] ] ==  3  && col[ row_major[6] ] ==  0;
    }
    //
    // Only interested in cross-terms. Since we are not computing rwo 0,
    // we do not need sparsity entries in row 0.
    CppAD::sparse_rc<SizeVector> subset_pattern(n, n, 3);
    for(size_t k = 0; k < 3; k++)
        subset_pattern.set(k, k+1, 0);
    CppAD::sparse_rcv<SizeVector, DoubleVector> subset( subset_pattern );
    //
    // argument and weight values for computation
    CPPAD_TESTVECTOR(double) x(n), w(m);
    for(size_t j = 0; j < n; j++)
        x[j] = double(n) / double(j+1);
    w[0] = 1.0;
    //
    std::string coloring = "cppad.general";
    size_t n_sweep = f.sparse_hes(
        x, w, subset, subset_pattern, coloring, work
    );
    ok &= n_sweep == 1;
    for(size_t k = 0; k < 3; k++)
    {   size_t i = k + 1;
        ok &= subset.val()[k] == double(i + 1);
    }
    //
    // convert subset from lower triangular to upper triangular
    for(size_t k = 0; k < 3; k++)
        subset_pattern.set(k, 0, k+1);
    subset = CppAD::sparse_rcv<SizeVector, DoubleVector>( subset_pattern );
    //
    // This will require more work because the Hessian is computed
    // column by column (not row by row).
    work.clear();
    n_sweep = f.sparse_hes(
        x, w, subset, subset_pattern, coloring, work
    );
    ok &= n_sweep == 3;
    //
    // but it will get the right answer
    for(size_t k = 0; k < 3; k++)
    {   size_t i = k + 1;
        ok &= subset.val()[k] == double(i + 1);
    }
    return ok;
}
// END C++
