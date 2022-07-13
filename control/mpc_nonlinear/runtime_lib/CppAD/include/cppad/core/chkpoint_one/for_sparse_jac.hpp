# ifndef CPPAD_CORE_CHKPOINT_ONE_FOR_SPARSE_JAC_HPP
# define CPPAD_CORE_CHKPOINT_ONE_FOR_SPARSE_JAC_HPP
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
template <class sparsity_type>
bool checkpoint<Base>::for_sparse_jac(
    size_t                                  q  ,
    const sparsity_type&                    r  ,
          sparsity_type&                    s  ,
    const vector<Base>&                     x  )
{   // make sure member_ is allocated for this thread
    size_t thread = thread_alloc::thread_num();
    allocate_member(thread);
    //
    // during user sparsity calculations
    size_t m = member_[thread]->f_.Range();
    size_t n = member_[thread]->f_.Domain();
    if( member_[thread]->jac_sparse_bool_.size() == 0 )
        set_jac_sparse_bool();
    if( member_[thread]->jac_sparse_set_.n_set() != 0 )
        member_[thread]->jac_sparse_set_.resize(0, 0);
    CPPAD_ASSERT_UNKNOWN( member_[thread]->jac_sparse_bool_.size() == m * n );
    CPPAD_ASSERT_UNKNOWN( member_[thread]->jac_sparse_set_.n_set() == 0 );
    CPPAD_ASSERT_UNKNOWN( r.size() == n * q );
    CPPAD_ASSERT_UNKNOWN( s.size() == m * q );
    //
    bool ok = true;
    for(size_t i = 0; i < m; i++)
    {   for(size_t k = 0; k < q; k++)
            s[i * q + k] = false;
    }
    // sparsity for  s = jac_sparse_bool_ * r
    for(size_t i = 0; i < m; i++)
    {   for(size_t k = 0; k < q; k++)
        {   // initialize sparsity for S(i,k)
            bool s_ik = false;
            // S(i,k) = sum_j J(i,j) * R(j,k)
            for(size_t j = 0; j < n; j++)
            {   bool J_ij = member_[thread]->jac_sparse_bool_[ i * n + j];
                bool R_jk = r[j * q + k ];
                s_ik |= ( J_ij & R_jk );
            }
            s[i * q + k] = s_ik;
        }
    }
    return ok;
}
template <class Base>
bool checkpoint<Base>::for_sparse_jac(
    size_t                                  q  ,
    const vectorBool&                       r  ,
          vectorBool&                       s  ,
    const vector<Base>&                     x  )
{   // make sure member_ is allocated for this thread
    size_t thread = thread_alloc::thread_num();
    allocate_member(thread);
    //
    return for_sparse_jac< vectorBool >(q, r, s, x);
}
template <class Base>
bool checkpoint<Base>::for_sparse_jac(
    size_t                                  q  ,
    const vector<bool>&                     r  ,
          vector<bool>&                     s  ,
    const vector<Base>&                     x  )
{   // make sure member_ is allocated for this thread
    size_t thread = thread_alloc::thread_num();
    allocate_member(thread);
    //
    return for_sparse_jac< vector<bool> >(q, r, s, x);
}
template <class Base>
bool checkpoint<Base>::for_sparse_jac(
    size_t                                  q  ,
    const vector< std::set<size_t> >&       r  ,
          vector< std::set<size_t> >&       s  ,
    const vector<Base>&                     x  )
{   // make sure member_ is allocated for this thread
    size_t thread = thread_alloc::thread_num();
    allocate_member(thread);
    //
    // during user sparsity calculations
    size_t m = member_[thread]->f_.Range();
    size_t n = member_[thread]->f_.Domain();
    if( member_[thread]->jac_sparse_bool_.size() != 0 )
        member_[thread]->jac_sparse_bool_.clear();
    if( member_[thread]->jac_sparse_set_.n_set() == 0 )
        set_jac_sparse_set();
    CPPAD_ASSERT_UNKNOWN( member_[thread]->jac_sparse_bool_.size() == 0 );
    CPPAD_ASSERT_UNKNOWN( member_[thread]->jac_sparse_set_.n_set() == m );
    CPPAD_ASSERT_UNKNOWN( member_[thread]->jac_sparse_set_.end()   == n );
    CPPAD_ASSERT_UNKNOWN( r.size() == n );
    CPPAD_ASSERT_UNKNOWN( s.size() == m );

    bool ok = true;
    for(size_t i = 0; i < m; i++)
        s[i].clear();

    // sparsity for  s = jac_sparse_set_ * r
    for(size_t i = 0; i < m; i++)
    {   // compute row i of the return pattern
        local::sparse::list_setvec::const_iterator set_itr(
            member_[thread]->jac_sparse_set_, i
        );
        size_t j = *set_itr;
        while(j < n )
        {   std::set<size_t>::const_iterator itr_j;
            const std::set<size_t>& r_j( r[j] );
            for(itr_j = r_j.begin(); itr_j != r_j.end(); itr_j++)
            {   size_t k = *itr_j;
                CPPAD_ASSERT_UNKNOWN( k < q );
                s[i].insert(k);
            }
            j = *(++set_itr);
        }
    }

    return ok;
}

} // END_CPPAD_NAMESPACE
# endif
