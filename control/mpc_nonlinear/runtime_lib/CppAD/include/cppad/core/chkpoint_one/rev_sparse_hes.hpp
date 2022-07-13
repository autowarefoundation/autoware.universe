# ifndef CPPAD_CORE_CHKPOINT_ONE_REV_SPARSE_HES_HPP
# define CPPAD_CORE_CHKPOINT_ONE_REV_SPARSE_HES_HPP
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
bool checkpoint<Base>::rev_sparse_hes(
    const vector<bool>&                     vx ,
    const vector<bool>&                     s  ,
          vector<bool>&                     t  ,
    size_t                                  q  ,
    const sparsity_type&                    r  ,
    const sparsity_type&                    u  ,
          sparsity_type&                    v  ,
    const vector<Base>&                     x  )
{   // make sure member_ is allocated for this thread
    size_t thread = thread_alloc::thread_num();
    allocate_member(thread);
    //
    size_t n = member_[thread]->f_.Domain();
# ifndef NDEBUG
    size_t m = member_[thread]->f_.Range();
# endif
    CPPAD_ASSERT_UNKNOWN( vx.size() == n );
    CPPAD_ASSERT_UNKNOWN(  s.size() == m );
    CPPAD_ASSERT_UNKNOWN(  t.size() == n );
    CPPAD_ASSERT_UNKNOWN(  r.size() == n * q );
    CPPAD_ASSERT_UNKNOWN(  u.size() == m * q );
    CPPAD_ASSERT_UNKNOWN(  v.size() == n * q );
    //
    bool ok        = true;

    // make sure hes_sparse_bool_ has been set
    if( member_[thread]->hes_sparse_bool_.size() == 0 )
        set_hes_sparse_bool();
    if( member_[thread]->hes_sparse_set_.n_set() != 0 )
        member_[thread]->hes_sparse_set_.resize(0, 0);
    CPPAD_ASSERT_UNKNOWN( member_[thread]->hes_sparse_bool_.size() == n * n );
    CPPAD_ASSERT_UNKNOWN( member_[thread]->hes_sparse_set_.n_set() == 0 );


    // compute sparsity pattern for T(x) = S(x) * f'(x)
    t = member_[thread]->f_.RevSparseJac(1, s);
# ifndef NDEBUG
    for(size_t j = 0; j < n; j++)
        CPPAD_ASSERT_UNKNOWN( vx[j] || ! t[j] )
# endif

    // V(x) = f'(x)^T * g''(y) * f'(x) * R  +  g'(y) * f''(x) * R
    // U(x) = g''(y) * f'(x) * R
    // S(x) = g'(y)

    // compute sparsity pattern for A(x) = f'(x)^T * U(x)
    bool transpose = true;
    sparsity_type a(n * q);
    a = member_[thread]->f_.RevSparseJac(q, u, transpose);

    // Need sparsity pattern for H(x) = (S(x) * f(x))''(x) * R,
    // but use less efficient sparsity for  f(x)''(x) * R so that
    // hes_sparse_set_ can be used every time this is needed.
    for(size_t i = 0; i < n; i++)
    {   for(size_t k = 0; k < q; k++)
        {   // initialize sparsity pattern for H(i,k)
            bool h_ik = false;
            // H(i,k) = sum_j f''(i,j) * R(j,k)
            for(size_t j = 0; j < n; j++)
            {   bool f_ij = member_[thread]->hes_sparse_bool_[i * n + j];
                bool r_jk = r[j * q + k];
                h_ik     |= ( f_ij & r_jk );
            }
            // sparsity for H(i,k)
            v[i * q + k] = h_ik;
        }
    }

    // compute sparsity pattern for V(x) = A(x) + H(x)
    for(size_t i = 0; i < n; i++)
    {   for(size_t k = 0; k < q; k++)
            // v[ i * q + k ] |= a[ i * q + k];
            v[ i * q + k ] = bool(v[ i * q + k]) | bool(a[ i * q + k]);
    }
    return ok;
}
template <class Base>
bool checkpoint<Base>::rev_sparse_hes(
    const vector<bool>&                     vx ,
    const vector<bool>&                     s  ,
          vector<bool>&                     t  ,
    size_t                                  q  ,
    const vectorBool&                       r  ,
    const vectorBool&                       u  ,
          vectorBool&                       v  ,
    const vector<Base>&                     x  )
{   // make sure member_ is allocated for this thread
    size_t thread = thread_alloc::thread_num();
    allocate_member(thread);
    //
    return rev_sparse_hes< vectorBool >(vx, s, t, q, r, u, v, x);
}
template <class Base>
bool checkpoint<Base>::rev_sparse_hes(
    const vector<bool>&                     vx ,
    const vector<bool>&                     s  ,
          vector<bool>&                     t  ,
    size_t                                  q  ,
    const vector<bool>&                     r  ,
    const vector<bool>&                     u  ,
          vector<bool>&                     v  ,
    const vector<Base>&                     x  )
{   // make sure member_ is allocated for this thread
    size_t thread = thread_alloc::thread_num();
    allocate_member(thread);
    //
    return rev_sparse_hes< vector<bool> >(vx, s, t, q, r, u, v, x);
}
template <class Base>
bool checkpoint<Base>::rev_sparse_hes(
    const vector<bool>&                     vx ,
    const vector<bool>&                     s  ,
          vector<bool>&                     t  ,
    size_t                                  q  ,
    const vector< std::set<size_t> >&       r  ,
    const vector< std::set<size_t> >&       u  ,
          vector< std::set<size_t> >&       v  ,
    const vector<Base>&                     x  )
{   // make sure member_ is allocated for this thread
    size_t thread = thread_alloc::thread_num();
    allocate_member(thread);
    //
    size_t n = member_[thread]->f_.Domain();
# ifndef NDEBUG
    size_t m = member_[thread]->f_.Range();
# endif
    CPPAD_ASSERT_UNKNOWN( vx.size() == n );
    CPPAD_ASSERT_UNKNOWN(  s.size() == m );
    CPPAD_ASSERT_UNKNOWN(  t.size() == n );
    CPPAD_ASSERT_UNKNOWN(  r.size() == n );
    CPPAD_ASSERT_UNKNOWN(  u.size() == m );
    CPPAD_ASSERT_UNKNOWN(  v.size() == n );
    //
    bool ok        = true;

    // make sure hes_sparse_set_ has been set
    if( member_[thread]->hes_sparse_bool_.size() != 0 )
        member_[thread]->hes_sparse_bool_.clear();
    if( member_[thread]->hes_sparse_set_.n_set() == 0 )
        set_hes_sparse_set();
    CPPAD_ASSERT_UNKNOWN( member_[thread]->hes_sparse_bool_.size() == 0 );
    CPPAD_ASSERT_UNKNOWN( member_[thread]->hes_sparse_set_.n_set() == n );
    CPPAD_ASSERT_UNKNOWN( member_[thread]->hes_sparse_set_.end()   == n );

    // compute sparsity pattern for T(x) = S(x) * f'(x)
    t = member_[thread]->f_.RevSparseJac(1, s);
# ifndef NDEBUG
    for(size_t j = 0; j < n; j++)
        CPPAD_ASSERT_UNKNOWN( vx[j] || ! t[j] )
# endif

    // V(x) = f'(x)^T * g''(y) * f'(x) * R  +  g'(y) * f''(x) * R
    // U(x) = g''(y) * f'(x) * R
    // S(x) = g'(y)

    // compute sparsity pattern for A(x) = f'(x)^T * U(x)
    // 2DO: change a to use INTERNAL_SPARSE_SET
    bool transpose = true;
    vector< std::set<size_t> > a(n);
    a = member_[thread]->f_.RevSparseJac(q, u, transpose);

    // Need sparsity pattern for H(x) = (S(x) * f(x))''(x) * R,
    // but use less efficient sparsity for  f(x)''(x) * R so that
    // hes_sparse_set_ can be used every time this is needed.
    for(size_t i = 0; i < n; i++)
    {   v[i].clear();
        local::sparse::list_setvec::const_iterator set_itr(
            member_[thread]->hes_sparse_set_, i
        );
        size_t j = *set_itr;
        while( j < n )
        {   std::set<size_t>::const_iterator itr_j;
            const std::set<size_t>& r_j( r[j] );
            for(itr_j = r_j.begin(); itr_j != r_j.end(); itr_j++)
            {   size_t k = *itr_j;
                v[i].insert(k);
            }
            j = *(++set_itr);
        }
    }
    // compute sparsity pattern for V(x) = A(x) + H(x)
    std::set<size_t>::const_iterator itr;
    for(size_t i = 0; i < n; i++)
    {   for(itr = a[i].begin(); itr != a[i].end(); itr++)
        {   size_t j = *itr;
            CPPAD_ASSERT_UNKNOWN( j < q );
            v[i].insert(j);
        }
    }

    return ok;
}

} // END_CPPAD_NAMESPACE
# endif
