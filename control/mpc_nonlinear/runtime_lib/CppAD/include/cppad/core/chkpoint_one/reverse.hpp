# ifndef CPPAD_CORE_CHKPOINT_ONE_REVERSE_HPP
# define CPPAD_CORE_CHKPOINT_ONE_REVERSE_HPP
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

// ---------------------------------------------------------------------------
template <class Base>
bool checkpoint<Base>::reverse(
    size_t                    q  ,
    const vector<Base>&       tx ,
    const vector<Base>&       ty ,
          vector<Base>&       px ,
    const vector<Base>&       py )
{   // make sure member_ is allocated for this thread
    size_t thread = thread_alloc::thread_num();
    allocate_member(thread);
    //

# ifndef NDEBUG
    size_t n = member_[thread]->f_.Domain();
    size_t m = member_[thread]->f_.Range();
# endif
    CPPAD_ASSERT_UNKNOWN( member_[thread]->f_.size_var() > 0 );
    CPPAD_ASSERT_UNKNOWN( n == tx.size() / (q+1) );
    CPPAD_ASSERT_UNKNOWN( m == ty.size() / (q+1) );
    CPPAD_ASSERT_UNKNOWN( tx.size() % (q+1) == 0 );
    CPPAD_ASSERT_UNKNOWN( ty.size() % (q+1) == 0 );
    CPPAD_ASSERT_UNKNOWN( px.size() == n * (q+1) );
    CPPAD_ASSERT_UNKNOWN( py.size() == m * (q+1) );
    bool ok  = true;

    // put proper forward mode coefficients in f_
# ifdef NDEBUG
    // compute forward results for orders zero through q
    member_[thread]->f_.Forward(q, tx);
# else
    size_t i, j, k;
    //
    // compute forward results for orders zero through q
    vector<Base> check_ty = member_[thread]->f_.Forward(q, tx);
    for(i = 0; i < m; i++)
    {   for(k = 0; k <= q; k++)
        {   j = i * (q+1) + k;
            CPPAD_ASSERT_UNKNOWN( check_ty[j] == ty[j] );
        }
    }
# endif
    // now can run reverse mode
    px = member_[thread]->f_.Reverse(q+1, py);

    // no longer need the Taylor coefficients in f_
    // (have to reconstruct them every time)
    size_t c = 0;
    size_t r = 0;
    member_[thread]->f_.capacity_order(c, r);
    return ok;
}
// ---------------------------------------------------------------------------
template <class Base>
bool checkpoint<Base>::reverse(
    size_t                          q   ,
    const vector< AD<Base> >&       atx ,
    const vector< AD<Base> >&       aty ,
          vector< AD<Base> >&       apx ,
    const vector< AD<Base> >&       apy )
{   // make sure member_ is allocated for this thread
    size_t thread = thread_alloc::thread_num();
    allocate_member(thread);
    //
    // make sure af_ is defined
    if( member_[thread]->af_.size_var() == 0 )
        member_[thread]->af_ = member_[thread]->f_.base2ad();
# ifndef NDEBUG
    size_t n = member_[thread]->f_.Domain();
    size_t m = member_[thread]->f_.Range();
# endif
    CPPAD_ASSERT_UNKNOWN( member_[thread]->f_.size_var() > 0 );
    CPPAD_ASSERT_UNKNOWN( n == atx.size() / (q+1) );
    CPPAD_ASSERT_UNKNOWN( m == aty.size() / (q+1) );
    CPPAD_ASSERT_UNKNOWN( atx.size() % (q+1) == 0 );
    CPPAD_ASSERT_UNKNOWN( aty.size() % (q+1) == 0 );
    CPPAD_ASSERT_UNKNOWN( apx.size() == n * (q+1) );
    CPPAD_ASSERT_UNKNOWN( apy.size() == m * (q+1) );
    bool ok  = true;

    // put proper forward mode coefficients in f_
# ifdef NDEBUG
    // compute forward results for orders zero through q
    member_[thread]->af_.Forward(q, atx);
# else
    size_t i, j, k;
    //
    // compute forward results for orders zero through q
    vector< AD<Base> > check_aty = member_[thread]->af_.Forward(q, atx);
    for(i = 0; i < m; i++)
    {   for(k = 0; k <= q; k++)
        {   j = i * (q+1) + k;
            CPPAD_ASSERT_UNKNOWN( check_aty[j] == aty[j] );
        }
    }
# endif
    // now can run reverse mode
    apx = member_[thread]->af_.Reverse(q+1, apy);

    // no longer need the Taylor coefficients in f_
    // (have to reconstruct them every time)
    size_t c = 0;
    size_t r = 0;
    member_[thread]->af_.capacity_order(c, r);
    return ok;
}

} // END_CPPAD_NAMESPACE
# endif
