# ifndef CPPAD_CORE_CHKPOINT_ONE_SET_HES_SPARSE_SET_HPP
# define CPPAD_CORE_CHKPOINT_ONE_SET_HES_SPARSE_SET_HPP
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
void checkpoint<Base>::set_hes_sparse_set(void)
{   // make sure member_ is allocated for this thread
    size_t thread = thread_alloc::thread_num();
    allocate_member(thread);
    //
    CPPAD_ASSERT_UNKNOWN( member_[thread]->hes_sparse_set_.n_set() == 0 );
    size_t n = member_[thread]->f_.Domain();
    size_t m = member_[thread]->f_.Range();
    //
    // set version of sparsity for vector of all ones
    vector<bool> all_one(m);
    for(size_t i = 0; i < m; i++)
        all_one[i] = true;

    // set version of sparsity for n by n idendity matrix
    local::sparse::list_setvec identity;
    identity.resize(n, n);
    for(size_t j = 0; j < n; j++)
    {   // Not using post_element because only adding one element per set
        identity.add_element(j, j);
    }

    // compute sparsity pattern for H(x) = sum_i f_i(x)^{(2)}
    bool transpose  = false;
    bool dependency = false;
    member_[thread]->f_.ForSparseJacCheckpoint(
        n, identity, transpose, dependency, member_[thread]->jac_sparse_set_
    );
    member_[thread]->f_.RevSparseHesCheckpoint(
        n, all_one, transpose, member_[thread]->hes_sparse_set_
    );
    CPPAD_ASSERT_UNKNOWN( member_[thread]->hes_sparse_set_.n_set() == n );
    CPPAD_ASSERT_UNKNOWN( member_[thread]->hes_sparse_set_.end()   == n );
    //
    // drop the forward sparsity results from f_
    member_[thread]->f_.size_forward_set(0);
}

} // END_CPPAD_NAMESPACE
# endif
