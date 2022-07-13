# ifndef CPPAD_CORE_CHKPOINT_ONE_REV_SPARSE_JAC_HPP
# define CPPAD_CORE_CHKPOINT_ONE_REV_SPARSE_JAC_HPP
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
bool checkpoint<Base>::rev_sparse_jac(
    size_t                                  q  ,
    const sparsity_type&                    rt ,
          sparsity_type&                    st ,
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
    CPPAD_ASSERT_UNKNOWN( rt.size() == m * q );
    CPPAD_ASSERT_UNKNOWN( st.size() == n * q );
    bool ok  = true;
    //
    // S = R * J where J is jacobian
    for(size_t i = 0; i < q; i++)
    {   for(size_t j = 0; j < n; j++)
        {   // initialize sparsity for S(i,j)
            bool s_ij = false;
            // S(i,j) = sum_k R(i,k) * J(k,j)
            for(size_t k = 0; k < m; k++)
            {   // sparsity for R(i, k)
                bool R_ik = rt[ k * q + i ];
                bool J_kj = member_[thread]->jac_sparse_bool_[ k * n + j ];
                s_ij     |= (R_ik & J_kj);
            }
            // set sparsity for S^T
            st[ j * q + i ] = s_ij;
        }
    }
    return ok;
}
template <class Base>
bool checkpoint<Base>::rev_sparse_jac(
    size_t                                  q  ,
    const vectorBool&                       rt ,
          vectorBool&                       st ,
    const vector<Base>&                     x  )
{   // make sure member_ is allocated for this thread
    size_t thread = thread_alloc::thread_num();
    allocate_member(thread);
    //
    return rev_sparse_jac< vectorBool >(q, rt, st, x);
}
template <class Base>
bool checkpoint<Base>::rev_sparse_jac(
    size_t                                  q  ,
    const vector<bool>&                     rt ,
          vector<bool>&                     st ,
    const vector<Base>&                     x  )
{   // make sure member_ is allocated for this thread
    size_t thread = thread_alloc::thread_num();
    allocate_member(thread);
    //
    return rev_sparse_jac< vector<bool> >(q, rt, st, x);
}
template <class Base>
bool checkpoint<Base>::rev_sparse_jac(
    size_t                                  q  ,
    const vector< std::set<size_t> >&       rt ,
          vector< std::set<size_t> >&       st ,
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
    CPPAD_ASSERT_UNKNOWN( rt.size() == m );
    CPPAD_ASSERT_UNKNOWN( st.size() == n );
    //
    bool ok  = true;
    //
    for(size_t j = 0; j < n; j++)
        st[j].clear();
    //
    // sparsity for  s = r * jac_sparse_set_
    // s^T = jac_sparse_set_^T * r^T
    for(size_t i = 0; i < m; i++)
    {   // i is the row index in r^T
        std::set<size_t>::const_iterator itr_i;
        const std::set<size_t>& r_i( rt[i] );
        for(itr_i = r_i.begin(); itr_i != r_i.end(); itr_i++)
        {   // k is the column index in r^T
            size_t k = *itr_i;
            CPPAD_ASSERT_UNKNOWN( k < q );
            //
            // i is column index in jac_sparse_set^T
            local::sparse::list_setvec::const_iterator set_itr(
                member_[thread]->jac_sparse_set_, i
            );
            size_t j = *set_itr;
            while( j < n )
            {   // j is row index in jac_sparse_set^T
                st[j].insert(k);
                j = *(++set_itr);
            }
        }
    }

    return ok;
}

} // END_CPPAD_NAMESPACE
# endif
