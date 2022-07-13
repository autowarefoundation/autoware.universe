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
$begin rev_jac_sparsity.cpp$$
$spell
    Jacobian
    Jac
    Dep
    Cpp
$$

$section Reverse Mode Jacobian Sparsity: Example and Test$$


$srcthisfile%0%// BEGIN C++%// END C++%1%$$

$end
*/
// BEGIN C++
# include <cppad/cppad.hpp>

bool rev_jac_sparsity(void)
{   bool ok = true;
    using CppAD::AD;
    typedef CPPAD_TESTVECTOR(size_t)     SizeVector;
    typedef CppAD::sparse_rc<SizeVector> sparsity;
    //
    // domain space vector
    size_t n = 2;
    CPPAD_TESTVECTOR(AD<double>) ax(n);
    ax[0] = 0.;
    ax[1] = 1.;

    // declare independent variables and start recording
    CppAD::Independent(ax);

    // range space vector
    size_t m = 3;
    CPPAD_TESTVECTOR(AD<double>) ay(m);
    ay[0] = ax[0];
    ay[1] = ax[0] * ax[1];
    ay[2] = ax[1];

    // create f: x -> y and stop tape recording
    CppAD::ADFun<double> f(ax, ay);

    // sparsity pattern for the identity matrix
    size_t nr     = m;
    size_t nc     = m;
    size_t nnz_in = m;
    sparsity pattern_in(nr, nc, nnz_in);
    for(size_t k = 0; k < nnz_in; k++)
    {   size_t r = k;
        size_t c = k;
        pattern_in.set(k, r, c);
    }
    // compute sparsite pattern for J(x) = F'(x)
    bool transpose       = false;
    bool dependency      = false;
    bool internal_bool   = false;
    sparsity pattern_out;
    f.rev_jac_sparsity(
        pattern_in, transpose, dependency, internal_bool, pattern_out
    );
    size_t nnz = pattern_out.nnz();
    ok        &= nnz == 4;
    ok        &= pattern_out.nr() == m;
    ok        &= pattern_out.nc() == n;
    {   // check results
        const SizeVector& row( pattern_out.row() );
        const SizeVector& col( pattern_out.col() );
        SizeVector col_major = pattern_out.col_major();
        //
        ok &= row[ col_major[0] ] ==  0  && col[ col_major[0] ] ==  0;
        ok &= row[ col_major[1] ] ==  1  && col[ col_major[1] ] ==  0;
        ok &= row[ col_major[2] ] ==  1  && col[ col_major[2] ] ==  1;
        ok &= row[ col_major[3] ] ==  2  && col[ col_major[3] ] ==  1;
    }
    // note that the transpose of the identity is the identity
    transpose     = true;
    internal_bool = true;
    f.rev_jac_sparsity(
        pattern_in, transpose, dependency, internal_bool, pattern_out
    );
    nnz  = pattern_out.nnz();
    ok  &= nnz == 4;
    ok  &= pattern_out.nr() == n;
    ok  &= pattern_out.nc() == m;
    {   // check results
        const SizeVector& row( pattern_out.row() );
        const SizeVector& col( pattern_out.col() );
        SizeVector row_major = pattern_out.row_major();
        //
        ok &= col[ row_major[0] ] ==  0  && row[ row_major[0] ] ==  0;
        ok &= col[ row_major[1] ] ==  1  && row[ row_major[1] ] ==  0;
        ok &= col[ row_major[2] ] ==  1  && row[ row_major[2] ] ==  1;
        ok &= col[ row_major[3] ] ==  2  && row[ row_major[3] ] ==  1;
    }
    return ok;
}
// END C++
