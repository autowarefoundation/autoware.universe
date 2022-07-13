# ifndef CPPAD_CORE_CHKPOINT_TWO_FOR_TYPE_HPP
# define CPPAD_CORE_CHKPOINT_TWO_FOR_TYPE_HPP
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
\file chkpoint_two/for_type.hpp
Second generation checkpoint type computation.
*/
/*!
Link from atomic_three to type calculation

\param parameter_x [in]
is the value of the parameters in the corresponding function call
afun(ax, ay).

\param type_x [in]
specifies which components of x are
constants, dynamics, and variables

\param type_y [out]
specifies which components of y are
constants, dynamics, and variables
*/
template <class Base>
bool chkpoint_two<Base>::for_type(
    const vector<Base>&          parameter_x ,
    const vector<ad_type_enum>&  type_x      ,
    vector<ad_type_enum>&        type_y      )
{   size_t nr  = jac_sparsity_.nr();
    size_t nnz = jac_sparsity_.nnz();
    const vector<size_t>& row( jac_sparsity_.row() );
    const vector<size_t>& col( jac_sparsity_.col() );
    //
    CPPAD_ASSERT_UNKNOWN( jac_sparsity_.nr() == type_y.size() );
    CPPAD_ASSERT_UNKNOWN( jac_sparsity_.nc() == type_x.size() );
    //
    // initialize type_y as constant_enum
    for(size_t i = 0; i < nr; ++i)
        type_y[i] = constant_enum;
    //
    // loop over entries in Dependency pattern
    for(size_t k = 0; k < nnz; ++k)
    {   size_t i  = row[k];
        size_t j  = col[k];
        type_y[i] = std::max(type_y[i], type_x[j]);
    }
    return true;
}

} // END_CPPAD_NAMESPACE

# endif
