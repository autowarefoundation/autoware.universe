# ifndef CPPAD_CORE_CHKPOINT_ONE_SET_JAC_SPARSE_SET_HPP
# define CPPAD_CORE_CHKPOINT_ONE_SET_JAC_SPARSE_SET_HPP
/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-19 Bradley M. Bell

CppAD is distributed under the terms of the
             Eclipse Public License Version 2.0.

This Source Code may also be made available under the following
Secondary License when the conditions for such availability set forth
in the Eclipse Public License, Version 2.0 are satisfied:
      GNU General Public License, Version 2.0 or later.
---------------------------------------------------------------------------- */

namespace CppAD { // BEGIN_CPPAD_NAMESPACE

template <class Base>
void checkpoint<Base>::set_jac_sparse_set(void)
{   // make sure member_ is allocated for this thread
    size_t thread = thread_alloc::thread_num();
    allocate_member(thread);
    //
    CPPAD_ASSERT_UNKNOWN( member_[thread]->jac_sparse_set_.n_set() == 0 );
    bool transpose  = false;
    bool dependency = true;
    size_t n = member_[thread]->f_.Domain();
    size_t m = member_[thread]->f_.Range();
    // Use the choice for forward / reverse that results in smaller
    // size for the sparsity pattern of all variables in the tape.
    if( n <= m )
    {   local::sparse::list_setvec identity;
        identity.resize(n, n);
        for(size_t j = 0; j < n; j++)
        {   // Not using post_element because only adding one element per set
            identity.add_element(j, j);
        }
        member_[thread]->f_.ForSparseJacCheckpoint(
            n, identity, transpose, dependency, member_[thread]->jac_sparse_set_
        );
        member_[thread]->f_.size_forward_set(0);
    }
    else
    {   local::sparse::list_setvec identity;
        identity.resize(m, m);
        for(size_t i = 0; i < m; i++)
        {   // Not using post_element because only adding one element per set
            identity.add_element(i, i);
        }
        member_[thread]->f_.RevSparseJacCheckpoint(
            m, identity, transpose, dependency, member_[thread]->jac_sparse_set_
        );
    }
    CPPAD_ASSERT_UNKNOWN( member_[thread]->f_.size_forward_set() == 0 );
    CPPAD_ASSERT_UNKNOWN( member_[thread]->f_.size_forward_bool() == 0 );
}

} // END_CPPAD_NAMESPACE
# endif
