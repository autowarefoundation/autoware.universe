/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-21 Bradley M. Bell

CppAD is distributed under the terms of the
             Eclipse Public License Version 2.0.

This Source Code may also be made available under the following
Secondary License when the conditions for such availability set forth
in the Eclipse Public License, Version 2.0 are satisfied:
      GNU General Public License, Version 2.0 or later.
---------------------------------------------------------------------------- */

/*
$begin sparse_rcv.cpp$$
$spell
    rc
    rcv
$$

$section sparse_rcv: Example and Test$$


$srcthisfile%0%// BEGIN C++%// END C++%1%$$

$end
*/

// BEGIN C++
# include <cppad/utility/sparse_rcv.hpp>
# include <vector>

bool sparse_rcv(void)
{   bool ok = true;
    typedef std::vector<size_t> SizeVector;
    typedef std::vector<double> ValueVector;

    // sparsity pattern for a 5 by 5 diagonal matrix
    size_t nr  = 5;
    size_t nc  = 5;
    size_t nnz = 5;
    CppAD::sparse_rc<SizeVector> pattern(nr, nc, nnz);
    for(size_t k = 0; k < nnz; k++)
    {   size_t r = nnz - k - 1; // reverse or column-major order
        size_t c = nnz - k - 1;
        pattern.set(k, r, c);
    }

    // sparse matrix
    CppAD::sparse_rcv<SizeVector, ValueVector> matrix(pattern);
    for(size_t k = 0; k < nnz; k++)
    {   double v = double(k);
        matrix.set(nnz - k - 1, v);
    }

    // row, column, and value vectors
    const SizeVector&  row( matrix.row() );
    const SizeVector&  col( matrix.row() );
    const ValueVector& val( matrix.val() );
    SizeVector col_major = matrix.col_major();

    // check row,  column, and value
    for(size_t k = 0; k < nnz; k++)
    {   ok &= row[ col_major[k] ] == k;
        ok &= col[ col_major[k] ] == k;
        ok &= val[ col_major[k] ] == double(k);
    }

    // pat
    const CppAD::sparse_rc<SizeVector> pat( matrix.pat() );
    ok &= nnz == pat.nnz();
    ok &= nr  == pat.nr();
    ok &= nc  == pat.nc();
    for(size_t k = 0; k < nnz; k++)
    {   ok &= row[k] == pat.row()[k];
        ok &= col[k] == pat.col()[k];
    }

    // create an empty matrix
    CppAD::sparse_rcv<SizeVector, ValueVector> other;
    ok &= other.nnz() == 0;
    ok &= other.nr()  == 0;
    ok &= other.nc()  == 0;

    // now swap other with matrix
    matrix.swap(other);
    ok &= matrix.nnz() == 0;
    ok &= matrix.nr()  == 0;
    ok &= matrix.nc()  == 0;
    for(size_t k = 0; k < nnz; k++)
    {   ok &= other.row()[ col_major[k] ] == k;
        ok &= other.col()[ col_major[k] ] == k;
        ok &= other.val()[ col_major[k] ] == double(k);
    }

    // now use the assignment statement
    matrix = other;
    ok    &= other.nr()  == matrix.nr();
    ok    &= other.nc()  == matrix.nc();
    ok    &= other.nnz() == matrix.nnz();
    for(size_t k = 0; k < nnz; k++)
    {   ok &= matrix.row()[k] == other.row()[k];
        ok &= matrix.col()[k] == other.col()[k];
        ok &= matrix.val()[k] == other.val()[k];
    }

    // now use the copy constructor
    CppAD::sparse_rcv<SizeVector, ValueVector> copy(matrix);
    ok    &= copy.nr()  == matrix.nr();
    ok    &= copy.nc()  == matrix.nc();
    ok    &= copy.nnz() == matrix.nnz();
    for(size_t k = 0; k < nnz; k++)
    {   ok &= matrix.row()[k] == copy.row()[k];
        ok &= matrix.col()[k] == copy.col()[k];
        ok &= matrix.val()[k] == copy.val()[k];
    }
    return ok;
}

// END C++
