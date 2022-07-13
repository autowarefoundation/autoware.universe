# ifndef CPPAD_CORE_CHKPOINT_TWO_JAC_SPARSITY_HPP
# define CPPAD_CORE_CHKPOINT_TWO_JAC_SPARSITY_HPP
/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-18 Bradley M. Bell

CppAD is distributed under the terms of the
             Eclipse Public License Version 2.0.

This Source Code may also be made available under the following
Secondary License when the conditions for such availability set forth
in the Eclipse Public License, Version 2.0 are satisfied:
      GNU General Public License, Version 2.0 or later.
---------------------------------------------------------------------------- */
namespace CppAD { // BEGIN_CPPAD_NAMESPACE
/*!
\file chkpoint_two/jac_sparsity.hpp
Second generation checkpoint Jacobian sparsity patterns.
*/
/*!
chkpoint_two to Jacobian sparsity calculations.

\param dependency [in]
This argument is ignored.
The return pattern is always a dependency pattern which is a correct,
but possibly not efficient, sparsity pattern.

\param parameter_x [in]
contains the values for arguments that are parameters.

\param type_x [in]
what is the type, in afun(ax, ay), for each component of x.

\param select_x [in]
which domain components to include in the dependency or sparsity pattern.
The index zero is used for parameters.

\param select_y [in]
which range components to include in the dependency or sparsity pattern.
The index zero is used for parameters.

\param pattern_out [out]
is the dependency or sparsity pattern.
*/
// BEGIN_PROTOTYPE
template <class Base>
bool chkpoint_two<Base>::jac_sparsity(
    const vector<Base>&                     parameter_x  ,
    const vector<ad_type_enum>&             type_x       ,
    bool                                    dependency   ,
    const vector<bool>&                     select_x     ,
    const vector<bool>&                     select_y     ,
    sparse_rc< vector<size_t> >&            pattern_out  )
// END_PROTOTYPE
{   CPPAD_ASSERT_UNKNOWN( jac_sparsity_.nr() == select_y.size() );
    CPPAD_ASSERT_UNKNOWN( jac_sparsity_.nc() == select_x.size() );

    // count number of non-zeros
    size_t nnz = jac_sparsity_.nnz();
    size_t nr  = jac_sparsity_.nr();
    size_t nc  = jac_sparsity_.nc();
    const vector<size_t>& row = jac_sparsity_.row();
    const vector<size_t>& col = jac_sparsity_.col();
    size_t nnz_out = 0;
    for(size_t k = 0; k < nnz; ++k)
    {   size_t i = row[k];
        size_t j = col[k];
        if( select_x[j] & select_y[i] )
            ++nnz_out;
    }

    // set the output sparsity pattern
    pattern_out.resize(nr, nc, nnz_out);
    size_t ell = 0;
    for(size_t k = 0; k < nnz; ++k)
    {   size_t i = row[k];
        size_t j = col[k];
        if( select_x[j] & select_y[i] )
            pattern_out.set(ell++, i, j);
    }
    CPPAD_ASSERT_UNKNOWN( ell == nnz_out );
    //
    return true;
}

} // END_CPPAD_NAMESPACE
# endif
