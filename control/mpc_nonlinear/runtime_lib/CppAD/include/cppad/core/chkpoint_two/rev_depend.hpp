# ifndef CPPAD_CORE_CHKPOINT_TWO_REV_DEPEND_HPP
# define CPPAD_CORE_CHKPOINT_TWO_REV_DEPEND_HPP
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
\file chkpoint_two/rev_depend.hpp
Second generation checkpoint type computation.
*/
/*!
Link from atomic_three to dependency calculation

\param parameter_x [in]
is the value of the parameters in the corresponding function call
afun(ax, ay).

\param type_x [in]
is the AD type for ax in the corresponding afun(ax, ay) call.

\param depend_x [out]
specifies which components of x affect the values of interest

\param depend_y [in]
specifies which components of y affect the vlaues of interest
*/
template <class Base>
bool chkpoint_two<Base>::rev_depend(
    const vector<Base>&         parameter_x ,
    const vector<ad_type_enum>& type_x      ,
    vector<bool>&               depend_x    ,
    const vector<bool>&         depend_y    )
{   size_t nc  = jac_sparsity_.nc();
    size_t nnz = jac_sparsity_.nnz();
    const vector<size_t>& row( jac_sparsity_.row() );
    const vector<size_t>& col( jac_sparsity_.col() );
    //
    CPPAD_ASSERT_UNKNOWN( jac_sparsity_.nr() == depend_y.size() );
    CPPAD_ASSERT_UNKNOWN( jac_sparsity_.nc() == depend_x.size() );
    //
    // initialize depend_x as false
    for(size_t j = 0; j < nc; ++j)
        depend_x[j] = false;
    //
    // loop over entries in Dependency pattern
    for(size_t k = 0; k < nnz; ++k)
    {   size_t i  = row[k];
        size_t j  = col[k];
        if( depend_y[i] )
            depend_x[j] = true;
    }
    return true;
}

} // END_CPPAD_NAMESPACE

# endif
