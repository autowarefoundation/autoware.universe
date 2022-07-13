# ifndef CPPAD_LOCAL_CPPAD_COLPACK_HPP
# define CPPAD_LOCAL_CPPAD_COLPACK_HPP
/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-16 Bradley M. Bell

CppAD is distributed under the terms of the
             Eclipse Public License Version 2.0.

This Source Code may also be made available under the following
Secondary License when the conditions for such availability set forth
in the Eclipse Public License, Version 2.0 are satisfied:
      GNU General Public License, Version 2.0 or later.
---------------------------------------------------------------------------- */
# if CPPAD_HAS_COLPACK

namespace CppAD { namespace local { // BEGIN_CPPAD_LOCAL_NAMESPACE
/*!
\file cppad_colpack.hpp
External interface to Colpack routines used by cppad.
*/
// ---------------------------------------------------------------------------
/*!
Link from CppAD to ColPack used for general sparse matrices.

This CppAD library routine is necessary because
<code>ColPack/ColPackHeaders.h</code> has a
<code>using namespace std</code> at the global level.

\param m [in]
is the number of rows in the sparse matrix

\param n [in]
is the nubmer of columns in the sparse matrix.

\param adolc_pattern [in]
This vector has size m,
<code>adolc_pattern[i][0]</code> is the number of non-zeros in row i.
For <code>j = 1 , ... , adolc_sparsity[i]<code>,
<code>adolc_pattern[i][j]</code> is the column index (base zero) for the
non-zeros in row i.

\param color [out]
is a vector with size m.
The input value of its elements does not matter.
Upon return, it is a coloring for the rows of the sparse matrix.
\n
\n
If for some i, <code>color[i] == m</code>, then
<code>adolc_pattern[i][0] == 0</code>.
Otherwise, <code>color[i] < m</code>.
\n
\n
Suppose two differen rows, <code>i != r</code> have the same color.
It follows that for all column indices j;
it is not the case that both
<code>(i, j)</code> and <code>(r, j)</code> appear in the sparsity pattern.
\n
\n
This routine tries to minimize, with respect to the choice of colors,
the number of colors.
*/
CPPAD_LIB_EXPORT void cppad_colpack_general(
          CppAD::vector<size_t>&         color         ,
    size_t                               m             ,
    size_t                               n             ,
    const CppAD::vector<unsigned int*>&  adolc_pattern
);

/*!
Link from CppAD to ColPack used for symmetric sparse matrices
(not yet used or tested).

This CppAD library routine is necessary because
<code>ColPack/ColPackHeaders.h</code> has a
<code>using namespace std</code> at the global level.

\param n [in]
is the nubmer of rows and columns in the symmetric sparse matrix.

\param adolc_pattern [in]
This vector has size n,
<code>adolc_pattern[i][0]</code> is the number of non-zeros in row i.
For <code>j = 1 , ... , adolc_sparsity[i]<code>,
<code>adolc_pattern[i][j]</code> is the column index (base zero) for the
non-zeros in row i.

\param color [out]
The input value of its elements does not matter.
Upon return, it is a coloring for the rows of the sparse matrix.
The properties of this coloring have not yet been determined; see
Efficient Computation of Sparse Hessians Using Coloring
and Automatic Differentiation (pdf/ad/gebemedhin14.pdf)
*/
CPPAD_LIB_EXPORT void cppad_colpack_symmetric(
          CppAD::vector<size_t>&         color         ,
    size_t                               n             ,
    const CppAD::vector<unsigned int*>&  adolc_pattern
);

} } // END_CPPAD_LOCAL_NAMESPACE

# endif
# endif

